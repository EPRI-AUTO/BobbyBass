import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from motor_control.robotcontrol import RobotControl
import math

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.target_waypoint_subscription = self.create_subscription(
            Point, '/target_waypoint', self.target_waypoint_callback, 10)
        self.current_position_subscription = self.create_subscription(
            Point, '/current_position', self.current_position_callback, 10)
        self.target_waypoint = None
        self.current_position = None
        self.current_heading = 0.0  # Assuming the robot's orientation is available

        # Initialize the motor control
        self.seekur = RobotControl("/dev/ttyUSB0", 115200)  # Update the port as needed

    def target_waypoint_callback(self, msg):
        self.target_waypoint = msg
        self.calculate_motor_commands()

    def current_position_callback(self, msg):
        self.current_position = msg
        self.calculate_motor_commands()

    def calculate_motor_commands(self):
        if self.target_waypoint is None or self.current_position is None:
            return

        # Calculate the difference between the current position and target waypoint
        dx = self.target_waypoint.x - self.current_position.x
        dy = self.target_waypoint.y - self.current_position.y

        # Calculate the desired heading (angle to the target waypoint)
        desired_heading = math.atan2(dy, dx)

        # Calculate the steering angle (difference between desired and current heading)
        steering_angle = desired_heading - self.current_heading

        # Normalize the steering angle to the range [-pi, pi]
        steering_angle = (steering_angle + math.pi) % (2 * math.pi) - math.pi

        # Set a constant speed (or calculate based on distance)
        speed = 1.0  # Example speed

        # Send motor commands
        self.send_motor_commands(speed, steering_angle)

    def send_motor_commands(self, speed, steering_angle):
        # Convert steering angle to motor commands
        if steering_angle > 0:
            self.seekur.left_motor("FORWARD", int(speed))
            self.seekur.right_motor("FORWARD", int(speed * (1 - abs(steering_angle))))
        elif steering_angle < 0:
            self.seekur.left_motor("FORWARD", int(speed * (1 - abs(steering_angle))))
            self.seekur.right_motor("FORWARD", int(speed))
        else:
            self.seekur.left_motor("FORWARD", int(speed))
            self.seekur.right_motor("FORWARD", int(speed))

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
