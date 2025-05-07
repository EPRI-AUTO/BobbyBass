import rclpy
import yaml
import os
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import transforms3d as tf_transformations
from tf2_ros import TransformBroadcaster

class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')

        self.proj = None
        self.initial_gps = None
        self.geofence_latlon = []
        self.geofence = []
        self.normalized_geofence = []
        self.current_position = None
        self.last_position = None
        self.tf_timer = self.create_timer(0.1, self.publish_tf)  # 10 Hz

        self.declare_parameter('lawnmower_step_size', 2.0)
        self.lawnmower_step_size = self.get_parameter('lawnmower_step_size').value

        yaml_path = os.path.join(
            get_package_share_directory('gps_receiver'),
            'geofence.yaml'
        )
        self.get_logger().info(f"Loading geofence.yaml from: {yaml_path}")

        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                self.geofence_latlon = data.get('geofence', [])
                self.get_logger().info(f"Loaded {len(self.geofence_latlon)} geofence points from YAML.")
        except Exception as e:
            self.get_logger().error(f"Failed to load geofence.yaml: {e}")

        self.subscription = self.create_subscription(NavSatFix, '/gps_data', self.listener_callback, 10)
        self.status_publisher = self.create_publisher(String, 'geofence_status', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        self.timer = self.create_timer(2.0, self.publish_markers)

        self.initial_gps_set = False
        self.waypoints_generated = False

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoint_queue = []
        self.current_goal_index = 0
        self.sending_goals = False

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Geofence Monitor Node started.')

    def listener_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude

        if self.proj is None:
            zone_number = int((lon + 180) / 6) + 1
            is_southern = lat < 0
            self.proj = Proj(proj='utm', zone=zone_number, ellps='WGS84', south=is_southern)
            self.get_logger().info(f"Initialized UTM projection with zone {zone_number}, southern hemisphere: {is_southern}")

        utm_x, utm_y = self.proj(lon, lat)

        if self.initial_gps is None:
            self.initial_gps = (utm_x, utm_y)
            self.initial_gps_set = True
            self.get_logger().info(f"Initial GPS coordinates set as origin: {self.initial_gps}")

            for point in self.geofence_latlon:
                gx, gy = self.proj(point['longitude'], point['latitude'])
                self.geofence.append(Point(x=gx, y=gy, z=0.0))

            self.normalized_geofence = [
                Point(x=p.x - self.initial_gps[0], y=p.y - self.initial_gps[1], z=0.0)
                for p in self.geofence
            ]

        normalized_x = utm_x - self.initial_gps[0]
        normalized_y = utm_y - self.initial_gps[1]
        self.current_position = (normalized_x, normalized_y)
        self.last_position = (normalized_x, normalized_y)

        status_msg = String()
        if self.is_inside_geofence(normalized_x, normalized_y):
            self.get_logger().info(f'Rover is inside the geofence: ({normalized_x:.2f}, {normalized_y:.2f})')
            status_msg.data = "inside"
        else:
            self.get_logger().warn(f'Rover is outside the geofence: ({normalized_x:.2f}, {normalized_y:.2f})')
            status_msg.data = "outside"

        self.status_publisher.publish(status_msg)

        if self.initial_gps_set and not self.waypoints_generated:
            self.waypoints_generated = True
            self.get_logger().info("Generating lawnmower waypoints.")

    def is_inside_geofence(self, x, y):
        num = len(self.normalized_geofence)
        if num < 3:
            self.get_logger().warn("Geofence has fewer than 3 points. Cannot determine containment.")
            return False

        j = num - 1
        inside = False
        for i in range(num):
            xi = self.normalized_geofence[i].x
            yi = self.normalized_geofence[i].y
            xj = self.normalized_geofence[j].x
            yj = self.normalized_geofence[j].y
            if ((yi > y) != (yj > y)) and \
                    (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi):
                inside = not inside
            j = i
        return inside

    def publish_tf(self):
        if self.last_position is None:
            return

        x, y = self.last_position

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


    def publish_markers(self):
        if self.initial_gps is None:
            return

        marker_array = MarkerArray()

        geofence_marker = Marker()
        geofence_marker.header.frame_id = "map"
        geofence_marker.header.stamp = self.get_clock().now().to_msg()
        geofence_marker.ns = "geofence"
        geofence_marker.id = 0
        geofence_marker.type = Marker.LINE_STRIP
        geofence_marker.action = Marker.ADD
        geofence_marker.scale.x = 0.5
        geofence_marker.color.a = 1.0
        geofence_marker.color.r = 0.0
        geofence_marker.color.g = 1.0
        geofence_marker.color.b = 0.0

        for point in self.normalized_geofence:
            geofence_marker.points.append(Point(x=point.x, y=point.y, z=0.0))

        if self.normalized_geofence:
            geofence_marker.points.append(geofence_marker.points[0])

        marker_array.markers.append(geofence_marker)

        gps_marker = Marker()
        gps_marker.header.frame_id = "map"
        gps_marker.header.stamp = self.get_clock().now().to_msg()
        gps_marker.ns = "gps_position"
        gps_marker.id = 1
        gps_marker.type = Marker.SPHERE
        gps_marker.action = Marker.ADD
        if self.current_position:
            gps_marker.pose.position.x = self.current_position[0]
            gps_marker.pose.position.y = self.current_position[1]
        gps_marker.pose.position.z = 0.0
        gps_marker.scale.x = 1.0
        gps_marker.scale.y = 1.0
        gps_marker.scale.z = 1.0
        gps_marker.color.a = 1.0
        gps_marker.color.r = 0.0
        gps_marker.color.g = 0.0
        gps_marker.color.b = 1.0
        marker_array.markers.append(gps_marker)

        if self.initial_gps_set and self.waypoints_generated:
            self.generate_lawnmower_markers(marker_array)

        self.marker_publisher.publish(marker_array)

    def generate_lawnmower_markers(self, marker_array):
        if not self.normalized_geofence:
            self.get_logger().warn("Normalized geofence points not available.")
            return

        min_x = min(p.x for p in self.normalized_geofence)
        max_x = max(p.x for p in self.normalized_geofence)
        min_y = min(p.y for p in self.normalized_geofence)
        max_y = max(p.y for p in self.normalized_geofence)

        self.get_logger().info(f"Geofence boundaries: min_x={min_x}, max_x={max_x}, min_y={min_y}, max_y={max_y}")

        marker_id = 2
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                if self.is_inside_geofence(x, y):
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "lawnmower_waypoints"
                    marker.id = marker_id
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = 0.0
                    marker.scale.x = 0.5
                    marker.scale.y = 0.5
                    marker.scale.z = 0.5
                    marker.color.a = 1.0

                    if marker_id == 2:
                        marker.scale.x = 1.0
                        marker.scale.y = 1.0
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                    else:
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 1.0

                    marker_array.markers.append(marker)
                    self.create_goal_from_marker(marker)
                    marker_id += 1
                y += self.lawnmower_step_size
            x += self.lawnmower_step_size

        if self.waypoint_queue:
            self.send_next_goal()

    def create_goal_from_marker(self, marker):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = marker.pose.position
        q = tf_transformations.euler.euler2quat(0, 0, 0)  # returns (w, x, y, z)
        pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        self.waypoint_queue.append(pose)

    def send_next_goal(self):
        if not self.waypoint_queue or self.sending_goals:
            return

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return

        if self.current_goal_index >= len(self.waypoint_queue):
            self.get_logger().info("All waypoints completed.")
            self.sending_goals = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.waypoint_queue[self.current_goal_index]
        self.get_logger().info(f"Sending goal #{self.current_goal_index + 1}: x={goal_msg.pose.pose.position.x:.2f}, y={goal_msg.pose.pose.position.y:.2f}")

        self.sending_goals = True
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().debug(f"Received feedback: {feedback.feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected!")
            self.sending_goals = False
            return

        self.get_logger().info("Goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("Goal succeeded. Moving to next.")
        self.current_goal_index += 1
        self.sending_goals = False
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    geofence_monitor = GeofenceMonitor()
    rclpy.spin(geofence_monitor)
    geofence_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
