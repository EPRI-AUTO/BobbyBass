import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSReceiver(Node):
    def __init__(self):
        super().__init__('gps_receiver')

        # Declare parameters for TCP connection
        self.declare_parameter('tcp_host', '172.19.176.1')  # Default IP address
        self.declare_parameter('tcp_port', 7777)  # Default port

        # Get parameter values
        self.tcp_host = self.get_parameter('tcp_host').value
        self.tcp_port = self.get_parameter('tcp_port').value

        # Create a socket and connect to the server
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect((self.tcp_host, self.tcp_port))
            self.get_logger().info(f"Connected to {self.tcp_host}:{self.tcp_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {self.tcp_host}:{self.tcp_port}: {e}")
            raise

        # Create a publisher for GPS data
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # Create a timer to receive data periodically
        self.timer = self.create_timer(0.1, self.receive_data)

    def receive_data(self):
        try:
            data = self.client_socket.recv(1024)
            if data:
                decoded_data = data.decode('utf-8')
                self.get_logger().debug(f"Raw data received: {decoded_data}")

                lines = decoded_data.splitlines()
                for line in lines:
                    if line.startswith('$GPGGA'):
                        self.get_logger().debug(f"Processing GPGGA sentence: {line}")
                        gps_data = self.parse_gpgga(line)
                        if gps_data:
                            msg = NavSatFix()
                            msg.latitude = gps_data['latitude']
                            msg.longitude = gps_data['longitude']
                            msg.altitude = gps_data['altitude']
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"Published GPS data: Latitude = {msg.latitude}, Longitude = {msg.longitude}, Altitude = {msg.altitude}")
            else:
                self.get_logger().warn("No data received, closing connection.")
                self.client_socket.close()
                self.destroy_node()
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error while receiving data: {e}")
            self.client_socket.close()
            self.destroy_node()
            rclpy.shutdown()

    def parse_gpgga(self, sentence):
        try:
            fields = sentence.split(',')
            if len(fields) < 10:
                self.get_logger().warn(f"Malformed GPGGA sentence: {sentence}")
                return None

            latitude = self.convert_to_decimal_degrees(fields[2], fields[3])
            longitude = self.convert_to_decimal_degrees(fields[4], fields[5])
            altitude = float(fields[9]) if fields[9] else 0.0

            return {
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude
            }
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error parsing GPGGA sentence: {e}")
            return None

    def convert_to_decimal_degrees(self, value, direction):
        """
        Correctly parse both latitude (DDMM.MMMM) and longitude (DDDMM.MMMM)
        """
        if not value or not direction:
            return 0.0

        try:
            if direction in ['N', 'S']:
                degrees = float(value[:2])
                minutes = float(value[2:])
            else:  # 'E' or 'W'
                degrees = float(value[:3])
                minutes = float(value[3:])

            decimal_degrees = degrees + (minutes / 60.0)

            if direction in ['S', 'W']:
                decimal_degrees *= -1

            return decimal_degrees
        except Exception as e:
            self.get_logger().error(f"Error converting to decimal degrees: {e}")
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    gps_receiver = GPSReceiver()
    rclpy.spin(gps_receiver)
    gps_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
