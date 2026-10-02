from glob import glob
import os

from setuptools import find_packages, setup

package_name = "camera_robot_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "robots"),
            glob("config/robots/*.yaml"),
        ),
        (os.path.join("share", package_name, "boards"), glob("boards/*")),
        (
            os.path.join("share", package_name, "calibrations"),
            glob("calibrations/*"),
        ),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ashwanth",
    maintainer_email="ashwanth@todo.todo",
    description="Niryo D455 ChArUco eye-on-base calibration (easy_handeye2 + MoveIt2)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "charuco_detector = camera_robot_calibration.charuco_detector_node:main",
            "handeye_moveit_sampler = camera_robot_calibration.handeye_moveit_sampler:main",
            "generate_charuco_board = camera_robot_calibration.generate_charuco_board:main",
            "click_move = camera_robot_calibration.click_move_node:main",
            "handeye_camera_link_publisher = camera_robot_calibration.handeye_camera_link_publisher:main",
            "aligned_color_cloud = camera_robot_calibration.aligned_color_cloud_node:main",
        ],
    },
)
