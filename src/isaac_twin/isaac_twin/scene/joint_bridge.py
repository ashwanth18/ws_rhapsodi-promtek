"""OmniGraph: Niryo joints <-> ros2_control TopicBasedSystem, plus /clock."""

from __future__ import annotations

import omni.graph.core as og
import usdrt.Sdf

GRAPH_PATH = "/World/TwinJointBridge"


def create_joint_bridge(
    articulation_root: str,
    states_topic: str = "isaac_joint_states",
    commands_topic: str = "isaac_joint_commands",
) -> None:
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubJoints", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("Controller", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "PubJoints.inputs:execIn"),
                ("OnTick.outputs:tick", "SubJoints.inputs:execIn"),
                ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
                ("OnTick.outputs:tick", "Controller.inputs:execIn"),
                ("Context.outputs:context", "PubJoints.inputs:context"),
                ("Context.outputs:context", "SubJoints.inputs:context"),
                ("Context.outputs:context", "PubClock.inputs:context"),
                ("SimTime.outputs:simulationTime", "PubJoints.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
                ("SubJoints.outputs:jointNames", "Controller.inputs:jointNames"),
                ("SubJoints.outputs:positionCommand", "Controller.inputs:positionCommand"),
                ("SubJoints.outputs:velocityCommand", "Controller.inputs:velocityCommand"),
                ("SubJoints.outputs:effortCommand", "Controller.inputs:effortCommand"),
            ],
            keys.SET_VALUES: [
                ("PubJoints.inputs:topicName", states_topic),
                ("SubJoints.inputs:topicName", commands_topic),
                ("PubJoints.inputs:targetPrim", [usdrt.Sdf.Path(articulation_root)]),
                ("Controller.inputs:targetPrim", [usdrt.Sdf.Path(articulation_root)]),
            ],
        },
    )
