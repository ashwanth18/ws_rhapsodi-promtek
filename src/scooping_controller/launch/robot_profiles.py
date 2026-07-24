import copy
import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _profiles_path():
    return (
        Path(get_package_share_directory("scooping_controller"))
        / "config"
        / "robots.yaml"
    )


def _load_profiles():
    with _profiles_path().open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)["robots"]


def robot_profile(robot_name):
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


