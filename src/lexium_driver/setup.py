import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lexium_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ashwanth",
    maintainer_email="todo@todo.todo",
    description="ROS 2 driver for the Schneider Lexium Cobot (JSON/TCP protocol).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lexium_driver = lexium_driver.lexium_driver_node:main",
        ],
    },
)
