import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config = os.path.join(
        os.getenv('HOME'), 'trimble_ws', 'src', 'gps_receiver', 'geofence.rviz'
    )

    return LaunchDescription([
        # Launch RViz2 with the correct config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # Launch geofence_monitor to handle GPS data & markers
        Node(
            package='gps_receiver',
            executable='geofence_monitor',
            name='geofence_monitor',
            output='screen'
        )
    ])

