import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from robotcontrol import RobotControl
import time
import pygame
import os  # Needed to start and stop controller.py

# Check if the robot is connected and if the controller is connected
ROBOT_CONNECTED = False
try:
    import serial
    try:
        test_serial = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        test_serial.close()
        ROBOT_CONNECTED = True
    except serial.SerialException:
        ROBOT_CONNECTED = False

except ImportError:
    ROBOT_CONNECTED = False


class MotorSubscriberNode(Node):
    def __init__(self):
        super().__init__('motor_subscriber_node')

    # --------------- New stuff ----------------
        self.stuck_counter = 0 # Just initializes the variables
        self.stuck_time = 0 # This one keeps track of the last time the close signal was sent
    # --------------- New stuff ----------------

        pygame.joystick.init()
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)

        self.obstacle_left = False
        self.obstacle_right = False
        self.boost = 1

        # Subscribe to mode switch topic
        self.mode_subscription = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10)

        # Subscribe to obstacle signal from LiDAR
        self.subscription = self.create_subscription(
            Int32,
            '/obstacle_signal',     # This is the topic, where LiDAR node sends signal to
            self.robot_callback,
            10)

        self.mode_publisher = self.create_publisher(String, '/robot_mode', 1)
        self.battery_publisher = self.create_publisher(Int32, 'battery_level', 10)

        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.mode = "manual"  # Default mode
        
        # Initialize robot motor control if connected
        if ROBOT_CONNECTED:
            self.robot = RobotControl("/dev/ttyACM0", 115200)
            self.get_logger().info("Robot connected! Ready for manual driving.")
        else:
            self.get_logger().warn("Robot NOT connected. Running in simulation mode.")

        self.get_logger().info("Motor Subscriber Node Started in Manual Mode.")
        self.last_action_time = time.time()  # Track last obstacle handling time

        if ROBOT_CONNECTED and self.mode == "manual":
            self.manual_set()

    def timer_callback(self):
        try:
            bat_val = self.robot.battery_check()
            msg = Int32()
            msg.data = bat_val
            self.battery_publisher.publish(msg)
            self.get_logger().info(f'Battery Level: {bat_val}')
        except Exception as e:
            self.get_logger().error(f"Failed to read battery: {e}")

    def mode_callback(self, msg):
        """ Callback to switch between manual and autonomous mode. """
        # TO SWITCH TO MANUAL run: ros2 topic pub /robot_mode std_msgs/msg/String "data: 'manual'"
        # TO SWITCH TO AUTONOMOUS run: ros2 topic pub /robot_mode std_msgs/msg/String "data: 'autonomous'"
        if msg.data in ["manual", "autonomous"]:
            self.mode = msg.data
            self.get_logger().info(f"Mode switched to: {self.mode.upper()}")
    
    def robot_callback(self, msg):
        # print(self.robot.battery_check())
        pygame.event.pump()  # Process joystick events
        msg2 = String()

        # End motor_subscriber when BUTTON is pressed
        if self.joystick.get_button(1):
            raise KeyboardInterrupt()

        # Single button mode switching (edge detection)
        button_state = self.joystick.get_button(3)
        if button_state and not self.last_button_state:  # Detect when button is first pressed
            if self.mode == "manual":
                msg2.data = "autonomous"
            else:
                msg2.data = "manual"
            self.mode_publisher.publish(msg2)

        self.last_button_state = button_state  # Store button state for next loop        

        # Callback to handle obstacle detection (only in autonomous mode).
        if self.mode == "autonomous":
            # --------------- New stuff ----------------
            current_time = time.time() # keeps track of current time
           # If the counter has been triggered too many times by the close signals, stop and back up (will change to actually redirect later)
            if msg.data not in [3, 4, 5, 6] and self.stuck_counter >= 4:
                self.robot.left_motor("STOP", 1)
                self.robot.right_motor("STOP", 1)
                time.sleep(0.5)
                self.robot.left_motor("BACKWARD", 1)
                self.robot.right_motor("BACKWARD", 1)
                self.get_logger().warn("Robot may be stuck, reversing...")
                time.sleep(2)
                self.robot.left_motor("STOP", 1)
                self.robot.right_motor("STOP", 1)
                time.sleep(1)
                self.stuck_counter = 0
            # --------------- New stuff ----------------

            if msg.data == 1: #Close Left
                if not self.obstacle_left and not self.obstacle_right:
                    self.obstacle_left = True
                    self.stuck_time = current_time
                    self.robot.left_motor("STOP", 1)
                    self.robot.right_motor("STOP", 1)
                    time.sleep(0.5)
                if self.obstacle_left:
                    self.robot.left_motor("FORWARD", 1)
                    self.robot.right_motor("BACKWARD", 1)
                    self.get_logger().warn("Obstacle detected on the LEFT! Turning away...")
                    # --------------- New stuff ----------------
                    # Makes sure that multiple signals don't cause the counter to shoot up immediately
                    if current_time -  self.stuck_time> 1:
                        self.stuck_counter += 1
                        self.stuck_time = current_time
                    # --------------- New stuff ----------------
            elif msg.data == 2: #Close Right
                if not self.obstacle_left and not self.obstacle_right:
                    self.obstacle_right = True
                    self.stuck_time = current_time
                    self.robot.left_motor("STOP", 1)
                    self.robot.right_motor("STOP", 1)
                    time.sleep(0.5)
                if self.obstacle_right:
                    self.robot.left_motor("BACKWARD", 1)
                    self.robot.right_motor("FORWARD", 1)
                    self.get_logger().warn("Obstacle detected on the RIGHT! Turning away...")
                    # --------------- New stuff ----------------
                    # Makes sure that multiple signals don't cause the counter to shoot up immediately
                    if current_time -  self.stuck_time> 1:
                        self.stuck_counter += 1
                        self.stuck_time = current_time
                    # --------------- New stuff ----------------
            elif msg.data == 3: #Medium Left
                self.stuck_counter = 0 # Resets counter since the robot is not in clsoe range anymore
                if self.obstacle_left:
                    self.robot.left_motor("FORWARD", 1)
                    self.robot.right_motor("BACKWARD", 1)
                    self.get_logger().warn("Obstacle detected on the LEFT! Turning away...")
                else:
                    self.robot.left_motor("FORWARD", 1)
                    self.robot.right_motor("FORWARD", 1)
                    self.get_logger().info("Possible object detected on the left. Moving slowly...")
            elif msg.data == 4: #Medium Right
                self.stuck_counter = 0 # Resets counter since the robot is not in clsoe range anymore
                if self.obstacle_right:
                    self.robot.left_motor("BACKWARD", 1)
                    self.robot.right_motor("FORWARD", 1)
                    self.get_logger().warn("Obstacle detected on the RIGHT! Turning away...")
                else:
                    self.robot.left_motor("FORWARD", 1)
                    self.robot.right_motor("FORWARD", 1)
                    self.get_logger().info("Possible object detected on the right. Moving slowly...")
            elif msg.data == 5: #Far Left
                self.stuck_counter = 0 # Resets counter since the robot is not in clsoe range anymore
                if self.obstacle_left or self.obstacle_right:
                    self.obstacle_left = False
                    self.obstacle_right = False
                    self.robot.left_motor("STOP", 1)
                    self.robot.right_motor("STOP", 1)
                    self.get_logger().info("Obstacle avoided. Reseting...")
                    time.sleep(0.5)
                self.robot.left_motor("FORWARD", 2)
                self.robot.right_motor("FORWARD", 2)
                self.get_logger().info("Possible object detected in the distance. Slowing down...")
            elif msg.data == 6: #Far Right
                self.stuck_counter = 0 # Resets counter since the robot is not in clsoe range anymore
                if self.obstacle_left or self.obstacle_right:
                    self.obstacle_left = False
                    self.obstacle_right = False
                    self.robot.left_motor("STOP", 1)
                    self.robot.right_motor("STOP", 1)
                    self.get_logger().info("Obstacle avoided. Reseting...")
                    time.sleep(0.5)
                self.robot.left_motor("FORWARD", 2)
                self.robot.right_motor("FORWARD", 2)
                self.get_logger().info("Possible object detected in the distance. Slowing down...")
            else:
                if self.obstacle_left or self.obstacle_right:
                    self.obstacle_left = False
                    self.obstacle_right = False
                    self.robot.left_motor("STOP", 1)
                    self.robot.right_motor("STOP", 1)
                    self.get_logger().info("Obstacle avoided. Reseting...")
                    time.sleep(0.5)
                self.robot.left_motor("FORWARD", 5)
                self.robot.right_motor("FORWARD", 5)
                self.get_logger().info("No obstacle detected. Moving forward.")
        elif self.mode == "manual":
            self.manual_set()

    def manual_set(self):
        if self.joystick.get_button(0):
            self.boost = 2
        else:
            self.boost = 1
        # Joystick input
        move = self.joystick.get_axis(1)
        turn = self.joystick.get_axis(2)

        if abs(move) < 0.1:
            move = 0
        if abs(turn) < 0.1:
            turn = 0
        
        leftSpeed = -move + turn
        rightSpeed = -move - turn

        if leftSpeed == 0:
            left_dir = "STOP"
        elif leftSpeed > 0:
            left_dir = "FORWARD"
        elif leftSpeed < 0:
            left_dir = "BACKWARD"
        if rightSpeed == 0:
            right_dir = "STOP"
        elif rightSpeed > 0:
            right_dir = "FORWARD"
        elif rightSpeed < 0:
            right_dir = "BACKWARD"

        if abs(leftSpeed) > 0.95:
            left_speed = 4
        elif abs(leftSpeed) > 0.5:
            left_speed = 3
        elif abs(leftSpeed) > 0.25:
            left_speed = 2
        else:
            left_speed = 1
        if abs(rightSpeed) > 0.95:
            right_speed = 4
        elif abs(rightSpeed) > 0.5:
            right_speed = 3
        elif abs(rightSpeed) > 0.25:
            right_speed = 2
        else:
            right_speed = 1

        # D-Pad inputs
        if self.joystick.get_hat(0) == (1, 1):
            left_dir = right_dir = "FORWARD"
            left_speed = 3
            right_speed = 1
        elif self.joystick.get_hat(0) == (-1, 1):
            left_dir = right_dir = "FORWARD"
            left_speed = 1
            right_speed = 3
        elif self.joystick.get_hat(0) == (0, 1):
            left_dir = right_dir = "FORWARD"
            left_speed = right_speed = 3
        elif self.joystick.get_hat(0) == (0, -1):
            left_dir = right_dir = "BACKWARD"
            left_speed = right_speed = 3
        elif self.joystick.get_hat(0) == (-1, 0):
            left_dir = "BACKWARD"
            right_dir = "FORWARD"
            left_speed = right_speed = 1
        elif self.joystick.get_hat(0) == (1, 0):
            left_dir = "FORWARD"
            right_dir = "BACKWARD"
            left_speed = right_speed = 1

        self.robot.left_motor(left_dir, left_speed*self.boost)
        self.robot.right_motor(right_dir, right_speed*self.boost)

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    pygame.quit()
    robot = RobotControl("/dev/ttyACM0", 115200)
    robot.left_motor("STOP", 1)
    robot.right_motor("STOP", 1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
