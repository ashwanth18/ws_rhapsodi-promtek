import math

import numpy as np

from isaac_twin.kinematics import Chain

URDF = """
<robot name="two">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="fixed_tip" type="fixed">
    <parent link="l2"/><child link="tip"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_planar_two_link():
    chain = Chain(URDF, "base", "tip")
    assert chain.joint_names == ["j1", "j2"]
    t = chain.tip_pose({"j1": math.pi / 2, "j2": -math.pi / 2})
    np.testing.assert_allclose(t[:3, 3], [0.1, 0.2, 0.1], atol=1e-12)
    np.testing.assert_allclose(t[:3, :3], np.eye(3), atol=1e-12)


def test_ik_round_trip():
    chain = Chain(URDF, "base", "tip")
    target = chain.tip_pose({"j1": 0.4, "j2": -0.9})
    q = chain.ik(target, seed=[0.0, 0.0])
    assert q is not None
    np.testing.assert_allclose(chain.tip_pose(dict(zip(chain.joint_names, q)))[:3, 3], target[:3, 3], atol=1e-4)


def test_ik_unreachable_is_none():
    target = np.eye(4)
    target[:3, 3] = [1.0, 0.0, 0.1]
    assert Chain(URDF, "base", "tip").ik(target) is None


def test_ik_respects_limits():
    urdf = URDF.replace('<axis xyz="0 0 1"/>\n  </joint>\n  <joint name="fixed_tip"', '<axis xyz="0 0 1"/><limit lower="-0.5" upper="0.5"/>\n  </joint>\n  <joint name="fixed_tip"')
    chain = Chain(urdf, "base", "tip")
    assert chain.upper[1] == 0.5
    assert chain.ik(chain.tip_pose({"j1": 0.0, "j2": 1.2}), seed=[0.0, 0.0]) is None


def test_origin_rpy_is_fixed_axis():
    urdf = URDF.replace('<origin xyz="0.2 0 0" rpy="0 0 0"/>', '<origin xyz="0 0 0" rpy="1.5707963267948966 0 1.5707963267948966"/>')
    t = Chain(urdf, "base", "l2").tip_pose({})
    # Rz(90) @ Rx(90): x -> y, y -> z, z -> x
    np.testing.assert_allclose(t[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(t[:3, :3] @ [0, 1, 0], [0, 0, 1], atol=1e-12)
