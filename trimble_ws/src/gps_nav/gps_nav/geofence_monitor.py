import rclpy
import yaml
import os
import time
import utm
import transforms3d as tf_transformations
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion, TransformStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.clock import Clock
from tf2_ros import TransformBroadcaster


class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')

        #self.tf_broadcaster = TransformBroadcaster(self)
        #self.tf_timer = self.create_timer(0.2, self.publish_tf)
        self.geofence = []
        self.current_position = None
        self.goal_x = 100
        self.goal_y = 100
        self.x_diff = 100
        self.y_diff = 100
        self.yaw_deg = 0.0
        self.goal_index = 2
        self.marker_array = MarkerArray()
        self.good2go = False
        self.waypoints_generated = False
        self.nav_mode = "continuous"
        self.stop_time = 0
        self.stop_check = False
        self.return_to_start = False
        self.max_id = 100000
        self.goal_switch_time = time.time()
        self.stop_interval = 5

        self.manual_goal_override = False

        self.declare_parameter('lawnmower_step_size', 1.2)
        self.lawnmower_step_size = self.get_parameter('lawnmower_step_size').value

        # Load geofence.yaml
        yaml_path = os.path.join(
            get_package_share_directory('gps_nav'),
            'config',
            'geofence.yaml'
        )

        # Convert GPS to (x,y) if applicable
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                raw_points = data.get('geofence', [])

                # Assume GPS if 20 < lat < 50 and -130 < lon < -60 (crude check)
                if 20.0 < raw_points[0]['x'] < 50.0 and -130.0 < raw_points[0]['y'] < -60.0:
                    # Convert GPS to UTM
                    self.geofence = self.convert_geofence_gps_to_xy(raw_points)
                    self.geofence = self.offset_utm_coords(self.geofence)
                    self.get_logger().info("Loaded geofence from GPS (converted to meters).")
                else:
                    self.geofence = [Point(x=p['x'], y=p['y'], z=0.0) for p in raw_points]
                    self.get_logger().info("Loaded geofence in local (x,y) meters.")

                self.get_logger().info(f"Loaded {len(self.geofence)} geofence points.")

                if self.geofence:
                    self.generate_lawnmower_markers()
                    self.waypoints_generated = True
                    self.get_logger().info("Generated lawnmower waypoints at startup.")
        
        except Exception as e:
            self.get_logger().error(f"Failed to load geofence.yaml: {e}")

        self.odom_subscription = self.create_subscription(
            Odometry, '/odometry/pointlio', self.odom_callback, 10
        )

        self.goal_pose_subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10
        )

        self.status_publisher = self.create_publisher(String, 'geofence_status', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        self.diff_publisher = self.create_publisher(Vector3, '/goal_diff', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)

        self.get_logger().info('Geofence Monitor Node (odometry-based) started.')

    def odom_callback(self, msg):
        current_time = time.time()
        #self.get_logger().info(f'Goal ID: {self.goal_index}')
        if not self.return_to_start:
            self.get_logger().info(f'Yaw: {self.yaw_deg:.2f}, Goal ID: {self.goal_index}')
        else:
            self.get_logger().info(f'Done')
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_position = (x, y)

        orientation = msg.pose.pose.orientation
        ox = orientation.x
        oy = orientation.y
        oz = orientation.z
        ow = orientation.w
        if ox != 0.0:
            roll, pitch, yaw = euler_from_quaternion([ox, oy, oz, ow])
            roll_deg = roll * (180/3.1415)
            pitch_deg = pitch * (180/3.1415)
            self.yaw_deg = yaw * (180/3.1415)
            #self.get_logger().info(f'Orientation Quaternion: x={ox:.2f}, y={oy:.2f}, z={oz:.2f}, w={ow:.2f}')
            #self.get_logger().info(f'Euler: roll={roll_deg:.2f}, pitch={pitch_deg:.2f}, yaw={yaw_deg:.2f}')
            #self.get_logger().info(f'Euler: yaw={self.yaw_deg:.2f}')

        # Publish TF if needed
        #self.publish_tf()

        # Geofence check
        status_msg = String()
        if self.is_inside_geofence(x, y):
            status_msg.data = "inside"
            #self.get_logger().info(f'Inside geofence: ({x:.2f}, {y:.2f})')
        else:
            status_msg.data = "outside"
            #self.get_logger().warn(f'Outside geofence: ({x:.2f}, {y:.2f})')
        self.status_publisher.publish(status_msg)

        #self.get_logger().info(f'Goal Position: {self.goal_x:.2f}, {self.goal_y:.2f}')
        self.x_diff = self.goal_x - x
        self.y_diff = self.goal_y - y
        #self.get_logger().info(f'Position_Difference: {self.x_diff:.2f}, {self.y_diff:.2f}')

        msg = Vector3()
        msg.x = self.x_diff
        msg.y = self.y_diff
        msg.z = self.yaw_deg  # Optional, can leave as 0
        self.diff_publisher.publish(msg)

        # Generate waypoints only once
        if not self.waypoints_generated:
            self.waypoints_generated = True
            self.get_logger().info("Generating lawnmower waypoints...")

        # If manual goal override is active
        if self.manual_goal_override:
            distance_to_goal = (self.x_diff**2 + self.y_diff**2) ** 0.5
            if distance_to_goal < 0.2:
                self.get_logger().info('Reached clicked goal. Resuming waypoint mission.')
                self.manual_goal_override = False  # Turn off override, clicked goal finished
                
                # Remove clicked goal marker
                self.marker_array.markers = [m for m in self.marker_array.markers if m.ns != "clicked_goal"]
                self.marker_publisher.publish(self.marker_array)

            return  # Exit the odom_callback here! Don't touch waypoint goal if still under manual override

        if self.nav_mode == "interval":
            timeout = 25 + self.stop_interval
        else:
            timeout = 25
        
        if len(self.marker_array.markers) > 3:
        #if self.waypoints_generated and self.goal_index < len(self.marker_array.markers):
            self.goal_x = self.marker_array.markers[self.goal_index].pose.position.x
            self.goal_y = self.marker_array.markers[self.goal_index].pose.position.y
            self.get_logger().info(f'Marker Coordinates: x = {self.goal_x:.2f}, y = {self.goal_y:.2f}')

            if ((current_time - self.goal_switch_time) > timeout) and not self.return_to_start:
                if self.goal_index < (len(self.marker_array.markers) - 1):
                    self.goal_index += 1
                    self.goal_switch_time = current_time
                else:
                    self.goal_index = 3
                    self.return_to_start = True
                
            if (self.x_diff**2 + self.y_diff**2) ** 0.5 > 0.3:
                self.good2go = True
            if self.good2go:
                if ((self.x_diff**2 + self.y_diff**2) ** 0.5 < 0.2) and not self.return_to_start:
                    if self.goal_index < (len(self.marker_array.markers) - 1):
                        if not self.stop_check:
                            self.stop_check = True
                            self.stop_time = current_time

                        if self.nav_mode == "interval":
                            if (current_time - self.stop_time) > self.stop_interval:
                                self.goal_index += 1
                                self.good2go = False
                                self.stop_check = False
                                self.goal_switch_time = current_time
                        else:
                            self.goal_index += 1
                            self.good2go = False
                            self.stop_check = False
                            self.goal_switch_time = current_time
                    else:
                        self.goal_index = 3
                        self.return_to_start = True

    def goal_pose_callback(self, msg):
        self.get_logger().info(f'Received new clicked goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.manual_goal_override = True

        # Draw a marker for the clicked goal
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "clicked_goal"
        marker.id = 9999
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0

        # Add to marker array and publish immediately
        # (clear existing clicked goal if already exists)
        self.marker_array.markers = [m for m in self.marker_array.markers if m.ns != "clicked_goal"]
        self.marker_array.markers.append(marker)
        self.marker_publisher.publish(self.marker_array)

    def is_inside_geofence(self, x, y):
        num = len(self.geofence)
        if num < 3:
            self.get_logger().warn("Geofence has fewer than 3 points.")
            return False

        j = num - 1
        inside = False
        for i in range(num):
            xi, yi = self.geofence[i].x, self.geofence[i].y
            xj, yj = self.geofence[j].x, self.geofence[j].y
            if ((yi > y) != (yj > y)) and \
               (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi):
                inside = not inside
            j = i
        return inside

    '''def publish_tf(self):
        if not self.current_position:
            return
        x, y = self.current_position

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)'''

    def publish_markers(self):
        self.marker_array.markers = []
        if not self.geofence:
            return

        geofence_marker = Marker()
        geofence_marker.header.frame_id = "map"
        geofence_marker.header.stamp = self.get_clock().now().to_msg()
        geofence_marker.ns = "geofence"
        geofence_marker.id = 0
        geofence_marker.type = Marker.LINE_STRIP
        geofence_marker.action = Marker.ADD
        geofence_marker.scale.x = 0.1
        geofence_marker.color.a = 1.0
        geofence_marker.color.g = 1.0

        geofence_marker.points = self.geofence + [self.geofence[0]]
        self.marker_array.markers.append(geofence_marker)

        if self.current_position:
            pos_marker = Marker()
            pos_marker.header.frame_id = "map"
            pos_marker.header.stamp = self.get_clock().now().to_msg()
            pos_marker.ns = "robot_pos"
            pos_marker.id = 1
            pos_marker.type = Marker.SPHERE
            pos_marker.action = Marker.ADD
            pos_marker.pose.position.x = self.current_position[0]
            pos_marker.pose.position.y = self.current_position[1]
            pos_marker.pose.position.z = 0.0
            pos_marker.scale.x = pos_marker.scale.y = pos_marker.scale.z = 0.2
            pos_marker.color.a = 1.0
            pos_marker.color.b = 1.0
            self.marker_array.markers.append(pos_marker)

        if self.waypoints_generated:
            #self.generate_lawnmower_markers()
            self.generate_manual_markers()

        self.marker_publisher.publish(self.marker_array)

    def generate_manual_markers(self):
        manual_positions = [
            #(2, 0.46, 1.22),
            #(3, 0.46, 2.74),
            #(4, 2.29, 2.74),
            #(5, 2.29, 1.22)

            (2, -0.04, 0.72),
            (3, 0.79, 1.99),
        # Add more (id, x, y) as needed
        ]
        for marker_id, x, y, in manual_positions:
            if self.is_inside_geofence(x, y):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "waypoints"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.2
                marker.color.a = 1.0
                
                if marker_id == self.goal_index:
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif marker_id < self.goal_index:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    if not self.return_to_start:
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 1.0
                    else:
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0

                self.marker_array.markers.append(marker)

    def generate_lawnmower_markers(self):
        min_x = min(p.x for p in self.geofence)
        max_x = max(p.x for p in self.geofence)
        min_y = min(p.y for p in self.geofence)
        max_y = max(p.y for p in self.geofence)

        #these shift the waypoints inside the geofence
        start_x = min_x + self.lawnmower_step_size / 2 
        start_y = min_y + self.lawnmower_step_size / 2
        
        #makes sure waypoints don't fall on geofence
        adjusted_max_x = max_x - self.lawnmower_step_size / 2
        adjusted_max_y = max_y - self.lawnmower_step_size / 2

        marker_id = 2
        x = start_x
        column_count = 0
        while x <= adjusted_max_x:
            if column_count % 2 == 0:
                y = start_y
                y_condition = lambda y: y <= adjusted_max_y
                y_step = self.lawnmower_step_size
            else:
                y = adjusted_max_y
                y_condition = lambda y: y >= start_y
                y_step = -self.lawnmower_step_size
            while y_condition(y):    
                if self.is_inside_geofence(x, y):
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "waypoints"
                    marker.id = marker_id
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = 0.0
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.2
                    marker.color.a = 1.0
                    
                    if marker_id == self.goal_index:
                        marker.scale.x = 0.2
                        marker.scale.y = 0.2
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                    elif marker_id < self.goal_index:
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                    else:
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 1.0
                    
                    #marker.color.r = marker.color.g = marker.color.b = 1.0
                    self.marker_array.markers.append(marker)
                    marker_id += 1
                y += y_step
            x += self.lawnmower_step_size
            column_count += 1
        
        self.max_id = marker_id - 1

    def convert_geofence_gps_to_xy(self, gps_points):
        xy_points = []
        for p in gps_points:
            utm_x, utm_y, _, _ = utm.from_latlon(p['x'], p['y']) # lat = x, lon = y
            xy_points.append(Point(x=utm_x, y=utm_y, z=0.0))
        return xy_points
    
    def offset_utm_coords(self, points):
        ref_x = points[0].x
        ref_y = points[0].y
        for pt in points:
            pt.x -= ref_x
            pt.y -= ref_y
        return points


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

