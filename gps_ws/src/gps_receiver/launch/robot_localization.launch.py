from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }, 
            'config/ekf.yaml'],
            remappings=[
                ('/imu/data', '/unilidar/imu')
            ]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'frequency': 10.0,
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': False,
                'wait_for_datum': False,
                'use_manual_datum': False,
                'broadcast_utm_transform': True,
                'publish_navsat_transform': True,
                'use_odometry_yaw': True,
            }],
            remappings=[
                ('/imu', '/unilidar/imu'),
                ('/gps/fix', '/gps_data'),
                ('/odometry/filtered', '/odometry/gps')
            ]
        )
    ])
