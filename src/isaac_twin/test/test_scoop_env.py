import numpy as np
import pytest

gymnasium = pytest.importorskip("gymnasium")

from isaac_twin.gym.scoop_env import DEFAULT_URDF, ScoopEnv  # noqa: E402


@pytest.fixture(scope="module")
def env():
    return ScoopEnv(check_reachability=False, max_scoops=5)


def test_spaces_and_reset(env):
    obs, info = env.reset(seed=0)
    assert env.observation_space.contains(obs)
    # The sloped back wall rises above the fill there, so some interior cells are dry.
    depth = obs["height"][env.powder.bed.mask]
    assert depth.max() == pytest.approx(env.fill_depth_m, abs=1e-4)
    assert (depth > 0).mean() > 0.5
    assert info["bed_g"] > 0 and info["target_g"] > 0


def test_clearance_violation_is_not_executed(env):
    env.reset(seed=0)
    bed = env.powder.bed_g
    _obs, reward, _te, _tr, info = env.step(env.action_space.high)
    assert not info["clearance_ok"] and not info["executed"]
    assert reward <= -env.violation_penalty
    assert env.powder.bed_g == pytest.approx(bed)


def test_heuristic_scoop_conserves_mass(env):
    env.reset(seed=0)
    bed = env.powder.bed_g
    plan = env.heuristic_plan()
    assert plan.success
    action = np.array([plan.offset_x, plan.offset_y, plan.offset_z], dtype=np.float32)
    assert env.action_space.contains(action)
    obs, reward, _te, _tr, info = env.step(action)
    assert info["executed"] and info["scooped_g"] > 0
    assert -1.0 <= reward <= 0.0
    assert bed + info["bed_change_g"] == pytest.approx(env.powder.bed_g)
    assert info["bed_change_g"] + info["scooped_g"] + info["table_g"] == pytest.approx(0.0, abs=1e-6)
    assert env.observation_space.contains(obs)


def test_truncates_at_max_scoops(env):
    env.reset(seed=0)
    for k in range(env.max_scoops):
        *_, truncated, _info = env.step(env.action_space.high)
        assert truncated == (k == env.max_scoops - 1)


def test_registered_env_passes_checker():
    import isaac_twin.gym  # noqa: F401
    from gymnasium.utils.env_checker import check_env

    check_env(gymnasium.make("IsaacTwin/Scoop-v0", check_reachability=False).unwrapped, skip_render_check=True)


@pytest.mark.skipif(not DEFAULT_URDF.is_file(), reason="twin URDF not generated (run_isaac_twin.sh)")
def test_reachability_gives_joint_observation():
    env = ScoopEnv(max_scoops=2)
    env.reset(seed=0)
    obs, *_rest, info = env.step(np.zeros(3, dtype=np.float32))
    assert info["reachable"] and info["executed"]
    assert np.abs(obs["joints"]).sum() > 0
