from setuptools import find_packages, setup
from glob import glob
import os


package_name = "realsense_snapshot_analyzer"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@example.com",
    description="Capture a synced RealSense image + pointcloud snapshot via service and analyze depth/3D statistics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "snapshot_analyzer = realsense_snapshot_analyzer.snapshot_analyzer_node:main",
        ],
    },
)



