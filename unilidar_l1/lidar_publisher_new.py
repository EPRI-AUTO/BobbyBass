import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Int32  # Message type for obstacle detection signal
import numpy as np

class LiDARPublisherNode(Node):
    def __init__(self):
        super().__init__('lidar_publisher_node')

        # Subscribe to LiDAR point cloud data
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',  # Unitree LiDAR topic
            self.lidar_callback,
            10)
        
        # Publisher to send obstacle detection signal
        self.obstacle_publisher = self.create_publisher(Int32, '/obstacle_signal', 10)      # This is the topic, where LiDAR node sends signal to

        self.get_logger().info("LiDAR Publisher Node Started. Waiting for point cloud data...")

    def lidar_callback(self, msg):
        # Convert PointCloud2 message to numpy array for faster processing
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        # Filter points within detection threshold (0.3m)
        distances = np.linalg.norm(points, axis=1)
        obstacle_indices = np.where(distances < 0.3)[0]

        signal = Int32()
        if len(obstacle_indices) > 0:
            x_value = points[obstacle_indices[-1]][0]  # Last detected obstacle
            signal.data = 1 if x_value < 0 else 2  # Left (1) / Right (2)
            self.get_logger().warn(f"Obstacle detected at x={x_value:.2f}, sending signal: {signal.data}")
        else:
            self.get_logger().info("No obstacle detected. Sending signal: 0")
            signal.data = 0
            
        self.obstacle_publisher.publish(signal)

def main(args=None):
    rclpy.init(args=args)
    node = LiDARPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
