import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from robotcontrol import RobotControl
import time
import pygame
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

robot = RobotControl("/dev/ttyACM0", 115200)
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

pygame.init()

screen = pygame.display.set_mode((500, 500))

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
            #self.robot = RobotControl("/dev/ttyACM0", 115200)
            self.get_logger().info("Robot connected! Ready for autonomous driving.")
        else:
            self.get_logger().warn("Robot NOT connected. Running in simulation mode.")

        self.get_logger().info("Motor Subscriber Node Started in Autonomous Mode.")
        self.last_action_time = time.time()  # Track last obstacle handling time

        # Start moving forward by default if in autonomous mode
        if ROBOT_CONNECTED and self.mode == "autonomous":
            robot.left_motor("FORWARD", 2)
            robot.right_motor("FORWARD", 2)

    def mode_callback(self, msg):
        """ Callback to switch between manual and autonomous mode. """
        # TO SWITCH TO MANUAL run: ros2 topic pub /robot_mode std_msgs/msg/String "data: 'manual'"
        # TO SWITCH TO  AUTONOMOUS run: ros2 topic pub /robot_mode std_msgs/msg/String "data: 'autonomous'"
        if msg.data in ["manual", "autonomous"]:
            self.mode = msg.data
            self.get_logger().info(f"Mode switched to: {self.mode.upper()}")

            if self.mode == "manual":
                # Stop autonomous mode
                if ROBOT_CONNECTED:
                    robot.left_motor("STOP", 1)
                    robot.right_motor("STOP", 1)

                # Start controller.py for manual control
                self.start_manual_control()

            elif self.mode == "autonomous":
                # Stop controller.py
                self.stop_manual_control()

                # Resume autonomous movement
                if ROBOT_CONNECTED:
                    robot.left_motor("FORWARD", 2)
                    robot.right_motor("FORWARD", 2)

    def obstacle_callback(self, msg):
        """ Callback to handle obstacle detection (only in autonomous mode). """
        if self.mode != "autonomous":
            return  # Ignore obstacles in manual mode

        current_time = time.time()
        cooldown_period = 2  # Wait time after avoiding an obstacle

        if msg.data == 1:
            if current_time - self.last_action_time > cooldown_period:
                self.get_logger().warn("Obstacle detected! Stopping and maneuvering...")
                self.stop_and_turn()
                self.last_action_time = time.time()
        else:
            if current_time - self.last_action_time > cooldown_period:
                self.get_logger().info("No obstacle detected. Moving forward.")
                if ROBOT_CONNECTED:
                    robot.left_motor("FORWARD", 2)
                    robot.right_motor("FORWARD", 2)

    def stop_and_turn(self):
        """ Stops the robot and turns to avoid the obstacle. """
        if not ROBOT_CONNECTED:
            self.get_logger().warn("Robot not connected. Simulating movement.")
            return

        # Stop motors
        robot.left_motor("STOP", 1)
        robot.right_motor("STOP", 1)
        time.sleep(1)

        # Reverse
        robot.left_motor("BACKWARD", 2)
        robot.right_motor("BACKWARD", 2)
        time.sleep(2)

        # Turn (Right)
        robot.left_motor("FORWARD", 2)
        robot.right_motor("BACKWARD", 2)
        time.sleep(1)

        # Stop again
        robot.left_motor("STOP", 1)
        robot.right_motor("STOP", 1)
        self.get_logger().info("Obstacle avoided. Ready to move again!")

    def start_manual_control(self):
        """ Starts controller.py for joystick-based manual control. """
        if self.controller_process is None:
            self.get_logger().info("Starting manual control (controller.py)...")
            self.controller_process = os.popen("python3 ~/ros2_ws/dunker_motor/src/motor_control/motor_control/controller.py") # Change directory if different

    def stop_manual_control(self):
        """ Stops controller.py if running. """
        if self.controller_process is not None:
            self.get_logger().info("Stopping manual control (controller.py)...")
            self.controller_process.close()
            self.controller_process = None

def manual_set():
    # Joystick input
    hor = pygame.joystick.Joystick(0).get_axis(0)
    vert = pygame.joystick.Joystick(0).get_axis(1)

    if abs(hor) < 0.1 and abs(vert) < 0.1:
        leftSpeed = 0
        rightSpeed = 0
    else:
        if hor < 0:
            rightSpeed = -vert
            leftSpeed = rightSpeed * (1 - abs(hor))
        else:
            leftSpeed = -vert
            rightSpeed = leftSpeed * (1 - abs(hor))

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

    if abs(leftSpeed) > 0.75:
        left_speed = 4
    elif abs(leftSpeed) > 0.5:
        left_speed = 3
    elif abs(leftSpeed) > 0.25:
        left_speed = 2
    else:
        left_speed = 1
    if abs(rightSpeed) > 0.75:
        right_speed = 4
    elif abs(rightSpeed) > 0.5:
        right_speed = 3
    elif abs(rightSpeed) > 0.25:
        right_speed = 2
    else:
        right_speed = 1

    # D-Pad inputs
    if pygame.joystick.Joystick(0).get_hat(0) == (0, 1):
        left_dir = right_dir = "FORWARD"
        left_speed = right_speed = 2
    if pygame.joystick.Joystick(0).get_hat(0) == (0, -1):
        left_dir = right_dir = "BACKWARD"
        left_speed = right_speed = 4
    if pygame.joystick.Joystick(0).get_hat(0) == (-1, 0):
        left_dir = "BACKWARD"
        right_dir = "FORWARD"
        left_speed = right_speed = 2
    if pygame.joystick.Joystick(0).get_hat(0) == (1, 0):
        left_dir = "FORWARD"
        right_dir = "BACKWARD"
        left_speed = right_speed = 2

    robot.left_motor(left_dir, left_speed)
    robot.right_motor(right_dir, right_speed)

def bump_detect():
    bump = [0, 0, 0, 0, 0, 0, 0, 0]

    # Bump sensor detection
    for i in [1, 2, 3, 4, 5, 6, 7, 8]:
        bump[i - 1] = robot.bump_sensor(i)

    if sum(bump) > 0:
        robot.left_motor("QUICK STOP", 1)
        robot.right_motor("QUICK STOP", 1)

        if bump[0] == 1 or bump[1] == 1:
            bumpDir1 = "FORWARD"
            bumpDir2 = "LEFT"
        elif bump[2] == 1 or bump[3] == 1:
            bumpDir1 = "FORWARD"
            bumpDir2 = "RIGHT"
        elif bump[4] == 1 or bump[5] == 1:
            bumpDir1 = "BACKWARD"
            bumpDir2 = "LEFT"
        elif bump[6] == 1 or bump[7] == 1:
            bumpDir1 = "BACKWARD"
            bumpDir2 = "RIGHT"

        robot.left_motor(bumpDir1, 1)
        robot.right_motor(bumpDir1, 1)
        time.sleep(4)
        robot.left_motor("STOP", 1)
        robot.right_motor("STOP", 1)
        time.sleep(1)

        if bumpDir2 == "LEFT":
            robot.left_motor("BACKWARD", 1)
            robot.right_motor("FORWARD", 1)
        else:
            robot.left_motor("FORWARD", 1)
            robot.right_motor("BACKWARD", 1)

        time.sleep(2)
        robot.left_motor("STOP", 1)
        robot.right_motor("STOP", 1)
        time.sleep(2)

        bumped = True
    else:
        bumped = False
        
    return bumped

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriberNode()

    run = True
    while run:
        manual_set()


    '''
    run = True
    while run:
        mode = False
        while not mode:
            if bump_detect():
                time.sleep(0.1)
            else:
                manual_set()
                    
            if pygame.joystick.Joystick(0).get_button(4):
                mode = True

            if pygame.joystick.Joystick(0).get_button(1):
                run = False

        while mode:
            if bump_detect():
                time.sleep(0.1)
            else:
                rclpy.spin_once(node)
                
            if pygame.joystick.Joystick(0).get_button(5):
                mode = False
            
            if pygame.joystick.Joystick(0).get_button(1):
                run = False
    
    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()
    '''
    
    '''
    try:
        mode = False
        while True:
            while not mode:
                if bump_detect():
                    time.sleep(0.1)
                else:
                    manual_set()
                    
                if pygame.joystick.Joystick(0).get_button(4):
                    mode = True
            while mode:
                if bump_detect():
                    time.sleep(0.1)
                else:
                    rclpy.spin_once(node)
                
                if pygame.joystick.Joystick(0).get_button(5):
                    mode = False

    except KeyboardInterrupt:
        pass
    '''
if __name__ == '__main__':
    main()
