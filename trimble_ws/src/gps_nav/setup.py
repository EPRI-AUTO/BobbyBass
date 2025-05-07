from setuptools import setup

package_name = 'gps_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_localization.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml', 'config/geofence.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='your@email.com',
    description='GPS and waypoint-based Nav2 integration with SLAM support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_receiver = gps_nav.gps_receiver:main',
            'gps_waypoint_manager = gps_nav.gps_waypoint_manager:main',
            'pointlio_odometry_publisher = gps_nav.pointlio_odometry_publisher:main',
            'pointlio_odometry_publisher_2 = gps_nav.pointlio_odometry_publisher_2:main',
            'geofence_monitor = gps_nav.geofence_monitor:main',
            'geofence_monitor_2 = gps_nav.geofence_monitor_2:main',
            'geofence_monitor_3 = gps_nav.geofence_monitor_3:main',
            'geofence_click_to_goal = gps_nav.geofence_click_to_goal:main'
        ],
    },
)
