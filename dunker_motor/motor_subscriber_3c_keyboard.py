import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Twist, Vector3
from robotcontrol import RobotControl
import time
import pygame
import math

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

        pygame.init()
        screen = pygame.display.set_mode((500, 500))

        self.obstacle_left = False
        self.obstacle_right = False
        self.boost = 1
        self.previous_mode = "manual"
        self.bat_history = []
        self.nav_backup = False
        self.x_diff = 0.0
        self.y_diff = 0.0
        self.heading = 0.0
        self.commit_time = 0
        self.hard_turn = False
        self.cam_obstacle = False

        # Subscribe to mode switch topic
        self.mode_subscription = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10)

        # Subscribe to obstacle signal from LiDAR
        self.obstacle_subscription = self.create_subscription(
            Int32,
            '/obstacle_signal',     # This is the topic, where LiDAR node sends signal to
            self.robot_callback,
            10)
        
        self.cmdvel_subscription = self.create_subscription(
            Vector3,
            '/goal_diff', self.cmdvel_callback,
            10)
        
        self.zed_subscription_subscription = self.create_subscription(
            Int32,
            '/zed_obstacle_signal', self.zed_callback,
            10)

        self.mode_publisher = self.create_publisher(String, '/robot_mode', 1)
        self.battery_publisher = self.create_publisher(Int32, '/battery_percentage', 10)

        timer_period = 5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.mode = "manual"  # Default mode
        
        self.status_message = ""
        self.status_subscription = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10)
        
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)

        # Initialize robot motor control if connected
        if ROBOT_CONNECTED:
            self.robot = RobotControl("/dev/ttyACM0", 115200)
            self.get_logger().info("Robot connected! Ready for manual driving.")
        else:
            self.get_logger().warn("Robot NOT connected. Running in simulation mode.")

        self.get_logger().info("Motor Subscriber Node Started in Manual Mode.")
        self.last_action_time = time.time()  # Track last obstacle handling time

        status_msg = String()
        status_msg.data = "Current Status: Running"
        self.status_publisher.publish(status_msg)


        if ROBOT_CONNECTED and self.mode == "manual":
            self.manual_set()

    def zed_callback(self, msg):
        if msg.data == 1:
            self.cam_obstacle = True
        else:
            self.cam_obstacle = False


    def timer_callback(self):
        pygame.event.pump()
        bat_val = self.robot.battery_check()   # Get battery reading from arduino
        print(bat_val)
        bat_val = bat_val * 1.02               # Correction factor
        bat_voltage = bat_val / 25.31          # convert to actual voltage
        
        self.bat_history.append(bat_voltage)   # Update voltage history and get average of last five values
        if len(self.bat_history) > 5:
            self.bat_history.pop(0)

        bat_avg = sum(self.bat_history)/len(self.bat_history)
        bat_avg = round(bat_avg, 1)

        if bat_avg > 26.6:
            bat_percentage = 100
        elif bat_avg > 26.3:
            bat_percentage = 50
        elif bat_avg > 26.2:
            bat_percentage = 40
        elif bat_avg > 25.9:
            bat_percentage = 30
        elif bat_avg > 25.6:
            bat_percentage = 20
        elif bat_percentage > 23:
            bat_percentage = 10
        else:
            bat_percentage = 0
        
        if len(self.bat_history) < 5:
            self.get_logger().info(f'Gathering battery data.')
        else:
            if bat_percentage <= 10:
                self.robot.battery_crit("ON")
            elif bat_percentage <= 30:
                self.robot.battery_crit("OFF")
                self.robot.ind_light(5, "ON")
            else:
                self.robot.ind_light(5, "OFF")

            self.get_logger().info(f'Battery Voltage: {bat_avg}  Battery Level: {bat_percentage}%')
            msg = Int32()
            msg.data = bat_percentage
            self.battery_publisher.publish(msg)

    def status_callback(self, msg):
        self.status_message = msg.data
        self.get_logger().info(f"Status update: {self.status_message}")
   

    def mode_callback(self, msg):
        pygame.event.pump() 
        """ Callback to switch between manual and autonomous mode. """
        # TO SWITCH TO MANUAL run: ros2 topic pub /robot_mode std_msgs/msg/String "data: 'manual'"
        # TO SWITCH TO AUTONOMOUS run: ros2 topic pub /robot_mode std_msgs/msg/String "data: 'autonomous'"
        if msg.data in ["manual", "autonomous", "waypoint"]:
            self.mode = msg.data
            self.get_logger().info(f"Mode switched to: {self.mode.upper()}")

        if self.mode == "manual":
            self.robot.ind_light(2, "ON")
            self.robot.ind_light(3, "OFF")
            self.robot.ind_light(4, "OFF")

        if self.mode == "autonomous":
            self.robot.ind_light(2, "OFF")
            self.robot.ind_light(3, "ON")
            self.robot.ind_light(4, "OFF")

        if self.mode == "waypoint":
            self.robot.ind_light(2, "OFF")
            self.robot.ind_light(3, "OFF")
            self.robot.ind_light(4, "ON")
    
    def cmdvel_callback(self, msg):
        pygame.event.pump() 
        distance_tolerance = 0.15
        heading_tolerance = 10
        
        self.x_diff = msg.x
        self.y_diff = msg.y

        target_heading = math.atan2(self.x_diff,self.y_diff)
        target_heading = math.degrees(target_heading)
        if target_heading < 0:
            target_heading += 360

        self.heading = msg.z
        if self.heading < 0:
            self.heading = -self.heading
        elif self.heading > 0:
            self.heading = 360-self.heading

        #self.get_logger().info(f"Received x_diff: {self.x_diff:.2f}, y_diff: {self.y_diff:.2f}, heading:{self.heading:.2f}")
        self.get_logger().info(f"Target heading: {target_heading:.2f}, Current Heading:{self.heading:.2f}")

        if self.mode != "waypoint" or not ROBOT_CONNECTED:
            return

        heading_difference = target_heading - self.heading
        if heading_difference > 180:
            heading_difference -= 360
        if heading_difference < -180:
            heading_difference += 360

        

        if (self.x_diff**2 + self.y_diff**2) ** 0.5 < distance_tolerance:
            self.robot.left_motor("STOP", 1)
            self.robot.right_motor("STOP", 1)
        elif abs(heading_difference) > 60:      #hard turn
            self.hard_turn = True
            if heading_difference > 0:
                self.robot.left_motor("FORWARD", 1)
                self.robot.right_motor("BACKWARD", 1)
            else:
                self.robot.left_motor("BACKWARD", 1)
                self.robot.right_motor("FORWARD", 1)
        elif abs(heading_difference) > heading_tolerance:    #soft turn
            if self.hard_turn:
                if heading_difference > 0:
                    self.robot.left_motor("FORWARD", 1)
                    self.robot.right_motor("BACKWARD", 1)
                else:
                    self.robot.left_motor("BACKWARD", 1)
                    self.robot.right_motor("FORWARD", 1)
            else:
                if heading_difference > 0:
                    self.robot.left_motor("FORWARD", 2)
                    self.robot.right_motor("FORWARD", 1)
                else:
                    self.robot.left_motor("FORWARD", 1)
                    self.robot.right_motor("FORWARD", 2)
        else:
            self.hard_turn = False
            self.robot.left_motor("FORWARD", 1)
            self.robot.right_motor("FORWARD", 1)

        
        '''
        else:
            if abs(heading_difference) < heading_tolerance:
                self.robot.left_motor("FORWARD", 1)
                self.robot.right_motor("FORWARD", 1)
            elif heading_difference > 0:
                self.robot.left_motor("FORWARD", 1)
                self.robot.right_motor("BACKWARD", 1)
            else:
                self.robot.left_motor("BACKWARD", 1)
                self.robot.right_motor("FORWARD", 1)
        '''
        
        

    def robot_callback(self, msg):
        pygame.event.pump()  # Process joystick events
        key = pygame.key.get_pressed()
        msg2 = String()
        current_time = time.time() # keeps track of current time

        if (msg.data == 1 or msg.data == 2) and self.mode != "manual":
            msg2.data = "autonomous"
            self.mode_publisher.publish(msg2)

        button_state_4 = key[pygame.K_q]
        if button_state_4 and not self.last_button_state_4:  # Detect when button is first pressed
                self.robot.stuck("OFF")
        self.last_button_state_4 = button_state_4  # Store button state for next loop 

        if self.mode == "manual":
            self.robot.ind_light(2, "ON")
        else:
            self.robot.ind_light(2, "OFF")

        # End motor_subscriber when BUTTON is pressed
        if key[pygame.K_x] or self.status_message == "Current Status: Emergency Stop":
            self.robot.left_motor("STOP", 1)
            self.robot.right_motor("STOP", 1)
            self.robot.ind_light(2, "OFF")
            pygame.quit
            rclpy.shutdown()
            raise KeyboardInterrupt()

        # Single button mode switching (edge detection)
        button_state_1 = key[pygame.K_1]
        if button_state_1 and not self.last_button_state_1:  # Detect when button is first pressed
            msg2.data = "waypoint"
            self.nav_backup = True
            self.mode_publisher.publish(msg2)
            print("waypoint")
        self.last_button_state_1 = button_state_1  # Store button state for next loop 

        button_state_2 = key[pygame.K_2]
        if button_state_2 and not self.last_button_state_2:  # Detect when button is first pressed
            msg2.data = "autonomous"
            self.nav_backup = False
            self.mode_publisher.publish(msg2)
            print("autonomous")
        self.last_button_state_2 = button_state_2  # Store button state for next loop   

        button_state_3 = key[pygame.K_3]
        if button_state_3 and not self.last_button_state_3:  # Detect when button is first pressed
            msg2.data = "manual"
            self.nav_backup = False
            self.mode_publisher.publish(msg2)
            print("manual")
        self.last_button_state_3 = button_state_3  # Store button state for next loop 

        # Callback to handle obstacle detection (only in autonomous mode).
        if self.mode == "autonomous":
            # --------------- New stuff ----------------
            
           # If the counter has been triggered too many times by the close signals, stop and back up (will change to actually redirect later)
            if msg.data not in [3, 4, 5, 6] and self.stuck_counter >= 4:
                self.robot.left_motor("STOP", 1)
                self.robot.right_motor("STOP", 1)
                time.sleep(0.5)
                if not self.cam_obstacle:
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
                    self.commit_time = current_time
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
                    self.commit_time = current_time
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
                    '''
                    if self.nav_backup and ((current_time - self.commit_time) > 3):
                        msg2 = String()
                        msg2.data = "waypoint"
                        self.mode_publisher.publish(msg2)
                    '''
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
                    '''
                    if self.nav_backup and ((current_time - self.commit_time) > 3):
                        msg2 = String()
                        msg2.data = "waypoint"
                        self.mode_publisher.publish(msg2)
                    '''
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

            if self.nav_backup and ((current_time - self.commit_time) > 3.5):
                self.robot.left_motor("STOP", 1)
                self.robot.right_motor("STOP", 1)
                msg2 = String()
                msg2.data = "waypoint"
                self.mode_publisher.publish(msg2)
        elif self.mode == "manual":
            self.manual_set()

    def manual_set(self):
        key = pygame.key.get_pressed()
        pygame.event.pump()

        if key[pygame.K_e]:
            self.boost = 2
        else:
            self.boost = 1
        # Joystick input
        move = 0
        turn = 0

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
        if key[pygame.K_w]:
            left_dir = right_dir = "FORWARD"
            left_speed = right_speed = 3
        elif key[pygame.K_s]:
            left_dir = right_dir = "BACKWARD"
            left_speed = right_speed = 3
        elif key[pygame.K_a]:
            left_dir = "BACKWARD"
            right_dir = "FORWARD"
            left_speed = right_speed = 1
        elif key[pygame.K_d]:
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
    pygame.quit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

