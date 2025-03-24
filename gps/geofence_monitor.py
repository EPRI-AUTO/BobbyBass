import rclpy
import yaml
import os
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pyproj import Proj
from ament_index_python.packages import get_package_share_directory


class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')

        # Initialize UTM projection with a default zone
        self.proj = Proj(proj='utm', zone=33, ellps='WGS84', south=False)  # Default zone 33 (Europe)
        self.initial_gps = None
        self.geofence = []
        self.normalized_geofence = []  # Store normalized geofence points

        # Parameter for lawnmower pattern step size
        self.declare_parameter('lawnmower_step_size', 2.0)
        self.lawnmower_step_size = self.get_parameter('lawnmower_step_size').value

        # Load geofence.yaml
        yaml_path = os.path.join(
            get_package_share_directory('gps_receiver'),
            'geofence.yaml'
        )
        self.get_logger().info(f"Loading geofence.yaml from: {yaml_path}")

        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                for point in data.get('geofence', []):
                    utm_x, utm_y = self.proj(point['longitude'], point['latitude'])
                    self.geofence.append(Point(x=utm_x, y=utm_y, z=0.0))
                self.get_logger().info(f"Loaded {len(self.geofence)} geofence points from YAML.")
        except Exception as e:
            self.get_logger().error(f"Failed to load geofence.yaml: {e}")
            self.geofence = []

        # ROS 2 interfaces
        self.subscription = self.create_subscription(NavSatFix, '/gps_data', self.listener_callback, 10)
        self.status_publisher = self.create_publisher(String, 'geofence_status', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish markers periodically
        self.timer = self.create_timer(2.0, self.publish_markers)

        # Store normalized GPS once available
        self.initial_gps_set = False
        self.waypoints_generated = False

        self.get_logger().info('Geofence Monitor Node started.')

    def listener_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude

        utm_x, utm_y = self.proj(lon, lat)

        if self.initial_gps is None:
            self.initial_gps = (utm_x, utm_y)
            self.initial_gps_set = True
            self.get_logger().info(f"Initial GPS coordinates set as origin: {self.initial_gps}")

            # Normalize geofence points relative to initial GPS
            self.normalized_geofence = [
                Point(x=p.x - self.initial_gps[0], y=p.y - self.initial_gps[1], z=0.0)
                for p in self.geofence
            ]

        normalized_x = utm_x - self.initial_gps[0]
        normalized_y = utm_y - self.initial_gps[1]

        status_msg = String()
        if self.is_inside_geofence(normalized_x, normalized_y):
            self.get_logger().info(f'Rover is inside the geofence: ({normalized_x:.2f}, {normalized_y:.2f})')
            status_msg.data = "inside"
        else:
            self.get_logger().warn(f'Rover is outside the geofence: ({normalized_x:.2f}, {normalized_y:.2f})')
            status_msg.data = "outside"

        self.status_publisher.publish(status_msg)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = normalized_x
        t.transform.translation.y = normalized_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Generate lawnmower waypoints after GPS origin is known
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

    def publish_markers(self):
        if self.initial_gps is None:
            return

        marker_array = MarkerArray()

        # Geofence marker
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
            geofence_marker.points.append(geofence_marker.points[0])  # Close the loop

        marker_array.markers.append(geofence_marker)

        # GPS marker
        gps_marker = Marker()
        gps_marker.header.frame_id = "map"
        gps_marker.header.stamp = self.get_clock().now().to_msg()
        gps_marker.ns = "gps_position"
        gps_marker.id = 1
        gps_marker.type = Marker.SPHERE
        gps_marker.action = Marker.ADD
        gps_marker.pose.position.x = 0.0  # Relative to initial GPS
        gps_marker.pose.position.y = 0.0
        gps_marker.pose.position.z = 0.0
        gps_marker.scale.x = 1.0
        gps_marker.scale.y = 1.0
        gps_marker.scale.z = 1.0
        gps_marker.color.a = 1.0
        gps_marker.color.r = 0.0
        gps_marker.color.g = 0.0
        gps_marker.color.b = 1.0
        marker_array.markers.append(gps_marker)

        # Lawnmower markers
        if self.initial_gps_set and self.waypoints_generated:
            self.generate_lawnmower_markers(marker_array)

        self.marker_publisher.publish(marker_array)

    def generate_lawnmower_markers(self, marker_array):
        if not self.normalized_geofence:
            self.get_logger().warn("Normalized geofence points not available.")
            return

        # Calculate geofence boundaries from normalized points
        min_x = min(p.x for p in self.normalized_geofence)
        max_x = max(p.x for p in self.normalized_geofence)
        min_y = min(p.y for p in self.normalized_geofence)
        max_y = max(p.y for p in self.normalized_geofence)

        self.get_logger().info(f"Geofence boundaries: min_x={min_x}, max_x={max_x}, min_y={min_y}, max_y={max_y}")

        marker_id = 2  # Start after geofence and GPS markers

        # Generate a grid of markers spaced 2x2 meters apart
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                # Check if the waypoint is inside the geofence
                if self.is_inside_geofence(x, y):
                    # Create a marker at this grid point
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

                    # Set a different color for marker 2
                    if marker_id == 2:
                        marker.scale.x = 1.0
                        marker.scale.y = 1.0
                        marker.color.r = 1.0  # Red
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                    else:
                        marker.color.r = 1.0  # White (default color)
                        marker.color.g = 1.0
                        marker.color.b = 1.0

                    marker_array.markers.append(marker)
                    marker_id += 1

                # Move to the next row
                y += self.lawnmower_step_size

            # Move to the next column
            x += self.lawnmower_step_size


def main(args=None):
    rclpy.init(args=args)
    geofence_monitor = GeofenceMonitor()
    rclpy.spin(geofence_monitor)
    geofence_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
