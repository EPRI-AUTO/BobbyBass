import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('gps_nav')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'frequency': 10.0,
                'use_odometry_yaw': False,
                'wait_for_datum': False,
                'broadcast_utm_transform': True,
                'publish_filtered_gps': True,
                'use_manual_datum': False,
                'zero_altitude': True
            }],
            remappings=[
                ('/gps/fix', '/gps_data'),
                ('/imu', '/zed/zed_node/imu/data'),
                ('/odometry/filtered', '/odometry/gps')
            ]
        )
    ])

