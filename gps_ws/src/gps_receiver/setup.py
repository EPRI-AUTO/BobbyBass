#Package Configuration for GPS Receiver System
#Key Features:
#- Defines ROS 2 package metadata
#- Specifies executable entry points
#- Includes data files (launch files, YAML config)

from setuptools import find_packages, setup

package_name = 'gps_receiver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch',
            ['launch/geofence_launch.py',
            'launch/gps_geofence_launch.py', 
            'launch/robot_localization.launch.py']),
        # Include geofence configuration
        ('share/' + package_name, ['geofence.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brittany',
    maintainer_email='brittany@todo.todo',
    description='GPS receiver node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Node executables
            'gps_receiver = gps_receiver.gps_receiver:main',
            'geofence_monitor = gps_receiver.geofence_monitor:main',
            'simple_waypoint_follower = gps_receiver.simple_waypoint_follower:main',
        ],
    },
)
