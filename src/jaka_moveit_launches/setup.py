from setuptools import find_packages, setup

package_name = "jaka_moveit_launches"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    zip_safe=True,
    maintainer="meddy",
    maintainer_email="medhanit@jaka.com",
    description="Gazebo + MoveIt launch utilities for JAKA MoveIt configs",
    license="BSD",
)
