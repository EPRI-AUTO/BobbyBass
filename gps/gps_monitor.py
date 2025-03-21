import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from std_msgs.msg import String


class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')

        # Define geofence area using GPS points (latitude, longitude)
        self.geofence = [
            Point(x=37.7749, y=-122.4194),  # Corner 1 (lat, lon)
            Point(x=37.7750, y=-122.4194),  # Corner 2
            Point(x=37.7750, y=-122.4193),  # Corner 3
            Point(x=37.7749, y=-122.4193),  # Corner 4
        ]

        # Subscription to gps_data topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps_data',
            self.listener_callback,
            10)

        # Publisher for geofence status
        self.status_publisher = self.create_publisher(String, 'geofence_status', 10)

        self.get_logger().info('Geofence Monitor Node started.')

    def listener_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude
        status_msg = String()

        if self.is_inside_geofence(lat, lon):
            self.get_logger().info(f'Rover is inside the geofence: ({lat}, {lon})')
            status_msg.data = "inside"
        else:
            self.get_logger().warn(f'Rover is outside the geofence: ({lat}, {lon})')
            status_msg.data = "outside"

        self.status_publisher.publish(status_msg)

    def is_inside_geofence(self, lat, lon):
        """
        Determine if a point is inside a polygon using the ray-casting algorithm.
        """
        num = len(self.geofence)
        j = num - 1
        inside = False

        for i in range(num):
            xi, yi = self.geofence[i].x, self.geofence[i].y
            xj, yj = self.geofence[j].x, self.geofence[j].y
            if ((yi > lon) != (yj > lon)) and \
                    (lat < (xj - xi) * (lon - yi) / (yj - yi + 1e-10) + xi):
                inside = not inside
            j = i

        return inside


def main(args=None):
    rclpy.init(args=args)
    geofence_monitor = GeofenceMonitor()
    rclpy.spin(geofence_monitor)
    geofence_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()