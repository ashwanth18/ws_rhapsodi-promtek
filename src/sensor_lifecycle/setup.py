from glob import glob
import os

from setuptools import setup


package_name = "sensor_lifecycle"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@example.com",
    description="Lifecycle wrappers for sensor processes (scale, micro-ROS).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scale_launcher = sensor_lifecycle.scale_launcher:main",
            "micro_ros_launcher = sensor_lifecycle.micro_ros_launcher:main",
        ],
    },
)


















