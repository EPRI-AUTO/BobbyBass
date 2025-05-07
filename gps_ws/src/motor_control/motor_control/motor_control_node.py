import rclpy
from rclpy.node import Node
from motor_control.motor_control import RobotControl

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.get_logger().info("Motor Control Node started")

        # Initialize the RobotControl class
        self.robot = RobotControl(device='/dev/ttyUSB0', speed=115200)  # Update device and speed as needed

        # Example: Check battery and log the value
        battery_value = self.robot.battery_check()
        self.get_logger().info(f"Battery value: {battery_value}")

        # Example: Control the left motor
        self.robot.left_motor(direction="FORWARD", speed=1)

    def destroy_node(self):
        # Clean up the serial connection
        self.robot.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
