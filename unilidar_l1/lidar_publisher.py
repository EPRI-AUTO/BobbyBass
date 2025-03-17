import rclpy
import time
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
        detection_threshold = 0.47
        detection_threshold_2 = 0.4

        # Find the first set of coordinates that triggered the detection
        triggering_points = [
            (x, y, z) for x, y, z in point_cloud if (x**2 + y**2 + z**2) ** 0.5 < detection_threshold
        ]

        # Check if an obstacle is detected
        obstacle_detected = bool(triggering_points)

        # Log the result
        signal = Int32()
        if obstacle_detected:
            most_recent_coordinates = triggering_points[-1]  # Last element in the list
            x_value = most_recent_coordinates[0]
            if x_value < 0:
                self.get_logger().warn("Obstacle detected on the left! Sending signal: 1")
                signal.data = 1
            else:
                self.get_logger().warn("Obstacle detected on the right! Sending signal: 2")
                signal.data = 2
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
