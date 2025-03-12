import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Int32  # Message type for obstacle detection signal

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
        # Convert PointCloud2 message to list of points
        point_cloud = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        # Check if any point is within the detection range (e.g., 0.3)
        detection_threshold = 0.3
        obstacle_detected = any(
            (x**2 + y**2 + z**2) ** 0.5 < detection_threshold for x, y, z in point_cloud
        )

        # Publish 1 if obstacle detected, 0 otherwise
        signal = Int32()
        signal.data = 1 if obstacle_detected else 0
        self.obstacle_publisher.publish(signal)

        # Log the result
        if obstacle_detected:
            self.get_logger().warn("Obstacle detected within 0.3 meters! Sending signal: 1")
        else:
            self.get_logger().info("No obstacle detected. Sending signal: 0")

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
