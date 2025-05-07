import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start GPS Receiver Node
        Node(
            package='gps_receiver',
            executable='gps_receiver',
            name='gps_receiver',
            output='screen'
        ),

        # Start Geofence Monitor Node
        Node(
            package='gps_receiver',
            executable='geofence_monitor',
            name='geofence_monitor',
            output='screen'
        ),

        # Start RViz2 with saved geofence config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.expanduser("~/trimble_ws/src/gps_receiver/geofence.rviz")],
            output='screen'
        )
    ])

