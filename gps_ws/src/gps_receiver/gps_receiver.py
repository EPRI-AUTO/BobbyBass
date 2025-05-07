import serial  # For serial communication
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class GPSReceiver(Node):
    def __init__(self):
        super().__init__('gps_receiver')

        # --- Serial Port Configuration ---
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.serial_port = serial.Serial(port, baudrate=baud, timeout=1.0)
            self.get_logger().info(f"Connected to serial port {port} at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port {port}: {e}")
            raise

        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)
        self.timer = self.create_timer(0.1, self.receive_data)

    def receive_data(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line.startswith('$GPGGA'):
                self.get_logger().debug(f"Processing GPGGA sentence: {line}")
                gps_data = self.parse_gpgga(line)
                if gps_data:
                    msg = NavSatFix()
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gps_link'
                    msg.latitude = gps_data['latitude']
                    msg.longitude = gps_data['longitude']
                    msg.altitude = gps_data['altitude']
                    
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.publisher_.publish(msg)
                    self.get_logger().info(
                        f"Published GPS data: Latitude={msg.latitude}, "
                        f"Longitude={msg.longitude}, Altitude={msg.altitude}"
                    )
        except Exception as e:
            self.get_logger().error(f"Error while reading from serial: {e}")

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
        if not value or not direction:
            return 0.0
        try:
            if direction in ['N', 'S']:
                degrees = float(value[:2])
                minutes = float(value[2:])
            else:
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
