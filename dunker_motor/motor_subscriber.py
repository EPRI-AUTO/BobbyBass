import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from robotcontrol import RobotControl
import time
import os  # Needed to start and stop controller.py

# Check if the robot is connected and if the controller is connected
# If one of these is not connected, the code still runs and it will only print output to terminal
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

        # Subscribe to obstacle signal from LiDAR
        self.subscription = self.create_subscription(
            Int32,
            '/obstacle_signal',     # This is the topic, where LiDAR node sends signal to
            self.obstacle_callback,
            10)
        
        # Subscribe to mode switch topic
        self.mode_subscription = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10)

        self.mode = "autonomous"  # Default mode
        self.controller_process = None  # Track controller.py process

        # Initialize robot motor control if connected
        if ROBOT_CONNECTED:
            self.robot = RobotControl("/dev/ttyACM0", 115200)
            self.get_logger().info("Robot connected! Ready for autonomous driving.")
        else:
            self.get_logger().warn("Robot NOT connected. Running in simulation mode.")

        self.get_logger().info("Motor Subscriber Node Started in Autonomous Mode.")
        self.last_action_time = time.time()  # Track last obstacle handling time

        # Start moving forward by default if in autonomous mode
        if ROBOT_CONNECTED and self.mode == "autonomous":
            self.robot.left_motor("FORWARD", 2)
            self.robot.right_motor("FORWARD", 2)

    def mode_callback(self, msg):
        """ Callback to switch between manual and autonomous mode. """
        """ TO SWITCH TO MANUAL run: ros2 topic pub /robot_mode std_msgs/msg/String "data: 'manual'" """
        """ TO SWITCH TO  AUTONOMOUS run: ros2 topic pub /robot_mode std_msgs/msg/String "data: 'autonomous'" """
        if msg.data in ["manual", "autonomous"]:
            self.mode = msg.data
            self.get_logger().info(f"🔄 Mode switched to: {self.mode.upper()}")

            if self.mode == "manual":
                # Stop autonomous mode
                if ROBOT_CONNECTED:
                    self.robot.left_motor("STOP", 1)
                    self.robot.right_motor("STOP", 1)

                # Start controller.py for manual control
                self.start_manual_control()

            elif self.mode == "autonomous":
                # Stop controller.py
                self.stop_manual_control()

                # Resume autonomous movement
                if ROBOT_CONNECTED:
                    self.robot.left_motor("FORWARD", 2)
                    self.robot.right_motor("FORWARD", 2)

    def obstacle_callback(self, msg):
        """ Callback to handle obstacle detection (only in autonomous mode). """
        if self.mode != "autonomous":
            return  # Ignore obstacles in manual mode

        current_time = time.time()
        cooldown_period = 2  # Wait time after avoiding an obstacle

        if msg.data == 1:
            if current_time - self.last_action_time > cooldown_period:
                self.get_logger().warn("Obstacle detected on the LEFT! Stopping and maneuvering...")
                self.stop_and_turn_right()
                self.last_action_time = time.time()
        if msg.data == 2:
            if current_time - self.last_action_time > cooldown_period:
                self.get_logger().warn("Obstacle detected on the RIGHT! Stopping and maneuvering...")
                self.stop_and_turn_left()
                self.last_action_time = time.time()
        else:
            if current_time - self.last_action_time > cooldown_period:
                self.get_logger().info("No obstacle detected. Moving forward.")
                if ROBOT_CONNECTED:
                    self.robot.left_motor("FORWARD", 2)
                    self.robot.right_motor("FORWARD", 2)

    def stop_and_turn_right(self):
        """ Stops the robot and turns to avoid the obstacle. """
        if not ROBOT_CONNECTED:
            self.get_logger().warn("Robot not connected. Simulating movement.")
            return

        # Stop motors
        self.robot.left_motor("STOP", 1)
        self.robot.right_motor("STOP", 1)
        time.sleep(1)

        # Reverse
        self.robot.left_motor("BACKWARD", 2)
        self.robot.right_motor("BACKWARD", 2)
        time.sleep(2)

        # Stop motors
        self.robot.left_motor("STOP", 1)
        self.robot.right_motor("STOP", 1)
        time.sleep(1)

        # Turn (Right)
        self.robot.left_motor("FORWARD", 2)
        self.robot.right_motor("BACKWARD", 2)
        time.sleep(2)

        # Stop again
        self.robot.left_motor("STOP", 1)
        self.robot.right_motor("STOP", 1)
        self.get_logger().info("Obstacle avoided. Ready to move again!")

    def stop_and_turn_left(self):
        """ Stops the robot and turns to avoid the obstacle. """
        if not ROBOT_CONNECTED:
            self.get_logger().warn("Robot not connected. Simulating movement.")
            return

        # Stop motors
        self.robot.left_motor("STOP", 1)
        self.robot.right_motor("STOP", 1)
        time.sleep(1)

        # Reverse
        self.robot.left_motor("BACKWARD", 2)
        self.robot.right_motor("BACKWARD", 2)
        time.sleep(2)

        # Stop motors
        self.robot.left_motor("STOP", 1)
        self.robot.right_motor("STOP", 1)
        time.sleep(1)

        # Turn (Left)
        self.robot.left_motor("BACKWARD", 2)
        self.robot.right_motor("FORWARD", 2)
        time.sleep(2)

        # Stop again
        self.robot.left_motor("STOP", 1)
        self.robot.right_motor("STOP", 1)
        self.get_logger().info("Obstacle avoided. Ready to move again!")

    def start_manual_control(self):
        """ Starts controller.py for joystick-based manual control. """
        if self.controller_process is None:
            self.get_logger().info("Starting manual control (controller.py)...")
            self.controller_process = os.popen("python3 ~/ros2_ws/dunker_motor/dunker_motor_ros2/src/motor_control/motor_control/controller.py") # Change directory if different

    def stop_manual_control(self):
        """ Stops controller.py if running. """
        if self.controller_process is not None:
            self.get_logger().info("Stopping manual control (controller.py)...")
            self.controller_process.close()
            self.controller_process = None

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    robot = RobotControl("/dev/ttyACM0", 115200)
    robot.left_motor("STOP", 1)
    robot.right_motor("STOP", 1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
