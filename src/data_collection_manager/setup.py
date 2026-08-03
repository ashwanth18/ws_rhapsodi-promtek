from setuptools import find_packages, setup

package_name = 'data_collection_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashwanth',
    maintainer_email='ashwanth@todo.todo',
    description='Data collection manager for lights-out experiments.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'data_collection_manager = data_collection_manager.manager_node:main',
            'weight_sim = data_collection_manager.weight_sim:main',
            'lightsout_sim = data_collection_manager.lightsout_sim:main',
            'retention_watchdog = data_collection_manager.retention_watchdog:main',
            'uplink_daemon = data_collection_manager.uplink_daemon:main',
        ],
    },
)


