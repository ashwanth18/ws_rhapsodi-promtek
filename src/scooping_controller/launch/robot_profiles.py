"""Load robot profiles and resolve the active robot for a launch."""
from __future__ import annotations

import copy
import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _image_profiles_path() -> Path | None:
    try:
        return (
            Path(get_package_share_directory("scooping_controller"))
            / "config"
            / "robots.yaml"
        )
    except Exception:
        return None


def _repo_robots_yaml() -> Path | None:
    try:
        candidate = Path(__file__).resolve().parents[1] / "config" / "robots.yaml"
        if candidate.is_file():
            return candidate
    except IndexError:
        pass
    return None


def _profiles_dir() -> Path | None:
    configured = os.environ.get("ROBOT_PROFILES_DIR", "").strip()
    if configured:
        path = Path(configured)
        if path.is_dir():
            return path.resolve()
    # Deploy-bundle mount used by compose/devices/*.yml
    bundle = Path("/ws/config/robots")
    if bundle.is_dir():
        return bundle.resolve()
    # Repo checkout for laptop work before an overlay is sourced.
    try:
        repo_robots = Path(__file__).resolve().parents[3] / "config" / "robots"
        if repo_robots.is_dir():
            return repo_robots.resolve()
    except IndexError:
        pass
    return None


def _load_profiles_mapping() -> dict:
    """Load robots from bundle dir (preferred) or the image/repo robots.yaml."""
    profiles_dir = _profiles_dir()
    if profiles_dir is not None:
        mapping: dict = {}
        for path in sorted(profiles_dir.glob("*.yaml")):
            raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            if "robots" in raw and isinstance(raw["robots"], dict):
                mapping.update(raw["robots"])
            elif isinstance(raw, dict) and raw:
                # Single-robot file: top-level is the profile itself, keyed by stem.
                mapping[path.stem] = raw
        if mapping:
            return mapping
    for candidate in (_image_profiles_path(), _repo_robots_yaml()):
        if candidate is not None and candidate.is_file():
            with candidate.open("r", encoding="utf-8") as stream:
                return yaml.safe_load(stream)["robots"]
    raise RuntimeError("No robot profiles found (bundle dir or robots.yaml)")


def _load_profiles():
    return _load_profiles_mapping()


def robot_profile(robot_name: str) -> dict:
    requested = robot_name.strip().lower()
    profiles = _load_profiles()
    for key, profile in profiles.items():
        aliases = {key, *(profile.get("aliases") or [])}
        if requested in aliases:
            result = copy.deepcopy(profile)
            result["key"] = key
            return result
    supported = sorted(
        alias
        for key, profile in profiles.items()
        for alias in {key, *(profile.get("aliases") or [])}
    )
    supported_values = ", ".join(supported)
    raise RuntimeError(
        f"Unsupported robot '{robot_name}'. Supported values: "
        f"{supported_values}."
    )


def _device_robot_type() -> str:
    """Read device.robot_type; prefer rhapsodi_common, fall back to YAML."""
    try:
        from rhapsodi_common.device_config import load_device_config

        return str(load_device_config().robot_type).strip().lower()
    except Exception:
        pass
    candidates = []
    env_value = os.environ.get("RHAPSODI_DEVICE_CONFIG")
    if env_value:
        candidates.append(Path(env_value))
    candidates.append(Path("/ws/config/device.yaml"))
    candidates.append(Path.cwd() / "config" / "device.yaml")
    for candidate in candidates:
        try:
            if not candidate.is_file():
                continue
            raw = yaml.safe_load(candidate.read_text(encoding="utf-8")) or {}
            device = raw.get("device") if isinstance(raw, dict) else None
            if isinstance(device, dict) and device.get("robot_type"):
                return str(device["robot_type"]).strip().lower()
        except (OSError, yaml.YAMLError):
            continue
    return ""


def _profile_robot_type(profile_id: str) -> str | None:
    """Return robot_type declared for PROFILE_ID in config/profiles.yaml."""
    if not profile_id:
        return None
    candidates = [
        Path(os.environ.get("PROFILES_YAML", "/ws/config/profiles.yaml")),
        Path("/ws/config/profiles.yaml"),
    ]
    # Repo checkout fallback for laptop launches.
    try:
        here = Path(__file__).resolve()
        candidates.append(here.parents[3] / "config" / "profiles.yaml")
    except IndexError:
        pass
    for path in candidates:
        try:
            if not path.is_file():
                continue
            raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            profiles = raw.get("profiles") if isinstance(raw, dict) else None
            if not isinstance(profiles, dict):
                continue
            entry = profiles.get(profile_id)
            if isinstance(entry, dict) and entry.get("robot_type"):
                return str(entry["robot_type"]).strip().lower()
        except (OSError, yaml.YAMLError):
            continue
    return None


def resolve_robot(explicit: str | None = None) -> str:
    """Resolve the robot key for a launch.

    Precedence:
      1. Explicit launch argument / ROBOT_TYPE env (non-empty)
      2. device.yaml robot_type
      3. Fallback 'niryo' (historical single-robot default)

    Refuses to start when PROFILE_ID's robot_type disagrees with the
    resolved device robot, or when the resolved profile is missing.
    """
    explicit_value = (explicit or "").strip().lower()
    if not explicit_value:
        explicit_value = os.environ.get("ROBOT_TYPE", "").strip().lower()

    device_type = _device_robot_type()
    resolved = explicit_value or device_type or "niryo"

    profile_id = os.environ.get("PROFILE_ID", "").strip()
    profile_type = _profile_robot_type(profile_id)
    if profile_type and device_type and profile_type != device_type:
        raise RuntimeError(
            f"PROFILE_ID '{profile_id}' declares robot_type '{profile_type}' "
            f"but device.yaml has robot_type '{device_type}'. Refusing to start."
        )
    if profile_type and resolved != profile_type and not explicit_value:
        # When no explicit override, prefer the device; still refuse mismatch
        # against the fleet profile.
        raise RuntimeError(
            f"Resolved robot '{resolved}' disagrees with PROFILE_ID "
            f"'{profile_id}' robot_type '{profile_type}'. Refusing to start."
        )

    # Validate the profile exists (raises RuntimeError if not).
    profile = robot_profile(resolved)
    return profile["key"]


def package_path(spec):
    return PathJoinSubstitution(
        [FindPackageShare(spec["package"]), spec["path"]]
    )


def package_share_path(spec):
    return os.path.join(
        get_package_share_directory(spec["package"]),
        spec["path"],
    )


def xacro_command_args(xacro_executable, urdf_spec):
    args = [xacro_executable, " ", package_path(urdf_spec)]
    for xacro_arg in urdf_spec.get("xacro_args", []):
        args.extend([" ", xacro_arg])
    return args


def default_targets_path(profile):
    return package_path(profile["targets"])
