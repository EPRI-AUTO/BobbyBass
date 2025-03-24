import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class BatteryListener(Node):
    def __init__(self):
        super().__init__('battery_listener')

        self.subscription = self.create_subscription(
            Int32,
            'battery_level',
            self.battery_callback,
            10
        )

        self.get_logger().info("Battery Listener Node started.")

    def battery_callback(self, msg):
        battery_level = msg.data
        self.get_logger().info(f'Received battery level: {battery_level}')

        if battery_level < 1000:  # <-- Set threshold
            self.get_logger().warn("Battery is low! Consider shutting down sensors.")
            # call shutdown functions here (e.g. disable lidar or camera)
        else:
            self.get_logger().info("Battery is healthy.")

def main(args=None):
    rclpy.init(args=args)
    battery_listener = BatteryListener()
    rclpy.spin(battery_listener)
    battery_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
