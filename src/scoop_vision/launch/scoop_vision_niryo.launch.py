#!/usr/bin/env python3
"""Native laptop session: Niryo scooping stack (RS6) + D455 + scoop_vision + RViz.

Same setup as the calibration sessions:
- everything runs from this workspace, so RViz resolves every mesh;
- the Niryo driver does not bridge /tf (the laptop robot_state_publisher owns
  the scoop URDF / tcp_link; see camera_robot_calibration issue log #2);
- layouts come from the repo's config/layouts (default dual-container → RS6).

Stop the Docker / Pi ROS stacks first: they share the DDS graph and publish
their own /tf, /robot_description and /cell_layout/active.

Run from the workspace root:
    ros2 launch scoop_vision scoop_vision_niryo.launch.py serial_no:=<D455 serial>
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    layouts_dir = LaunchConfiguration("layouts_dir").perform(context).strip()
    if not layouts_dir:
        layouts_dir = os.path.join(os.getcwd(), "config", "layouts")
    layouts_dir = os.path.abspath(os.path.expanduser(layouts_dir))
    layout_id = LaunchConfiguration("layout_id").perform(context).strip()
    if not os.path.isfile(os.path.join(layouts_dir, f"{layout_id}.yaml")):
        raise RuntimeError(
            f"{layout_id}.yaml not found in {layouts_dir}; run from the workspace "
            "root or pass layouts_dir:=<ws>/config/layouts"
        )

    no_tf = os.path.join(
        get_package_share_directory("camera_robot_calibration"), "config", "niryo_driver_no_tf.yaml"
    )
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scooping_controller"), "launch", "scooping_real.launch.py"
            )
        ),
        launch_arguments={
            "robot": "niryo",
            "layouts_dir": layouts_dir,
            "layout_id": layout_id,
            "whitelist_params_file": no_tf,
            "use_rviz": LaunchConfiguration("stack_rviz").perform(context),
        }.items(),
    )
    vision_args = {
        "layouts_dir": layouts_dir,
        "use_rviz": LaunchConfiguration("use_rviz").perform(context),
        "calibration_name": LaunchConfiguration("calibration_name").perform(context),
    }
    serial = LaunchConfiguration("serial_no").perform(context).strip()
    if serial:
        vision_args["serial_no"] = serial
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("scoop_vision"), "launch", "scoop_vision.launch.py")
        ),
        launch_arguments=vision_args.items(),
    )
    return [stack, vision]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # Keep this session off other machines' graphs (Pi on the LAN).
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("layout_id", default_value="dual-container"),
            DeclareLaunchArgument(
                "layouts_dir", default_value="", description="Default: $PWD/config/layouts"
            ),
            DeclareLaunchArgument("calibration_name", default_value="niryo_d455_eob"),
            DeclareLaunchArgument("use_rviz", default_value="true", description="scoop_vision RViz"),
            DeclareLaunchArgument(
                "stack_rviz",
                default_value="false",
                description="Also open the scooping controller RViz (scoop panel)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
