import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, Quaternion, Point, TransformStamped

class PointLIOOdometryPublisher(Node):
    def __init__(self):
        super().__init__('pointlio_odometry_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.publisher = self.create_publisher(Odometry, '/odometry/pointlio', 10)
        self.timer = self.create_timer(0.05, self.publish_odometry)  # 20Hz


    def publish_odometry(self):
        try:
            now = self.get_clock().now()
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            odom_msg = Odometry()
            #odom_msg.header.stamp = now.to_msg()
            odom_msg.header.stamp = trans.header.stamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position = Point(
                x=trans.transform.translation.x,
                y=trans.transform.translation.y,
                z=trans.transform.translation.z
            )
            odom_msg.pose.pose.orientation = trans.transform.rotation
            self.publisher.publish(odom_msg)
            
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation = trans.transform.translation
            t.transform.rotation = trans.transform.rotation
            #self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointLIOOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

