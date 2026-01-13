from setuptools import setup
import os
from glob import glob

package_name = 'niryo_robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='MoveIt2 control node for Niryo NED3 Pro robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_control_node = niryo_robot_control.moveit_control_node:main',
        ],
    },
) 