#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('lidar_status_publisher')
    pub = node.create_publisher(String, '/robot_status', 10)

    msg = String()
    msg.data = "LiDAR Initialized"
    pub.publish(msg)

    # Give it a moment to ensure message is sent
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
