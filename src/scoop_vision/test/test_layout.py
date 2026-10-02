import pytest
from conftest import REPO

from scoop_vision.layout import resolve_scene_path, task_container_mesh

LAYOUTS = REPO / "config/layouts"


def test_container_path_falls_back_to_local_layouts_dir():
    path = resolve_scene_path("/ws/config/layouts/lightsout-single-vessel.yaml", str(LAYOUTS))
    assert path == str(LAYOUTS / "lightsout-single-vessel.yaml")
    with pytest.raises(FileNotFoundError):
        resolve_scene_path("/ws/config/layouts/lightsout-single-vessel.yaml")


@pytest.mark.parametrize(
    "layout, task, mesh",
    [("dual-container", "rs6", "rs6_container.STL"), ("lightsout-single-vessel", "rs3", "rs3_container.STL")],
)
def test_task_container_mesh(layout, task, mesh):
    resource, scale = task_container_mesh(str(LAYOUTS / f"{layout}.yaml"), task)
    assert resource.endswith(mesh)
    assert scale == pytest.approx(0.001)


def _world(obj_pos, yaw_deg, p):
    import math
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return (obj_pos[0] + c * p[0] - s * p[1], obj_pos[1] + s * p[0] + c * p[1], obj_pos[2] + p[2])


def test_layout_proposal_moves_only_the_task_container(tmp_path, authored_poses):
    from types import SimpleNamespace

    import yaml

    from scoop_vision.layout import write_layout_proposal

    import numpy as np

    # numpy scalars, as fit_container_offset arithmetic can produce them.
    fit = SimpleNamespace(shift_x_m=np.float64(0.054), shift_y_m=np.float64(0.005),
                          z_offset_m=np.float64(0.007), yaw_offset_deg=np.float64(0.2),
                          rim_mad_at_best_m=np.float64(0.012))
    src = LAYOUTS / "dual-container.yaml"
    poses = [(p.position, p.orientation) for p in authored_poses]
    out = write_layout_proposal(str(src), "rs6", fit, str(tmp_path), current_poses=poses)

    old = yaml.safe_load(src.read_text())
    new = yaml.safe_load(open(out["layout"]).read())
    o_rs6 = next(o for o in old["objects"] if o["id"] == "rs6")
    n_rs6 = next(o for o in new["objects"] if o["id"] == "rs6")
    # Container +x is base -x (yaw ~180): the bin comes 54 mm toward the robot.
    ox, oy, oz = o_rs6["position_xyz"]
    assert n_rs6["position_xyz"] == pytest.approx([ox - 0.054, oy - 0.005, oz + 0.007], abs=1e-3)
    yaw0 = o_rs6["orientation"]["rpy_deg"][2]
    assert (n_rs6["orientation"]["rpy_deg"][2] - yaw0 - 0.2 + 180) % 360 - 180 == pytest.approx(0, abs=1e-6)
    assert [o for o in new["objects"] if o["id"] != "rs6"] == [o for o in old["objects"] if o["id"] != "rs6"]
    assert "# scoop_vision rim fit" in open(out["layout"]).read()
    assert src.read_text() == (LAYOUTS / "dual-container.yaml").read_text()  # source untouched

    # Re-anchored poses land on the same world points.
    moved = yaml.safe_load(open(out["poses_keep_world_path"]).read())["markers"]
    for (p_old, _), m in zip(poses, moved):
        p_new = [m["pose"]["position"][k] for k in "xyz"]
        a = _world(o_rs6["position_xyz"], o_rs6["orientation"]["rpy_deg"][2], p_old)
        b = _world(n_rs6["position_xyz"], n_rs6["orientation"]["rpy_deg"][2], p_new)
        assert a == pytest.approx(b, abs=1e-5)
