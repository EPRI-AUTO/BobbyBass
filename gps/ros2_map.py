import serial
import time
import socket

# Define the serial port and parameters
SERIAL_PORT = 'COM8'  # Change to match your system (e.g., '/dev/ttyUSB0' for Linux)
BAUD_RATE = 115200  # Default for Trimble AG 372
TIMEOUT = 1
TCP_HOST = '0.0.0.0'  # Listen on all interfaces
TCP_PORT = 7777  # The port the Ubuntu system will connect to

# Open the serial port
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

# Set up the TCP server to send data
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((TCP_HOST, TCP_PORT))
server_socket.listen(1)
print(f"Waiting for a connection on {TCP_HOST}:{TCP_PORT}...")

# Accept a connection from Ubuntu (client)
client_socket, client_address = server_socket.accept()
print(f"Connection established with {client_address}.")


# Function to validate NMEA checksum
def validate_nmea_checksum(sentence):
    if '*' not in sentence:
        return False
    data, checksum = sentence.split('*', 1)
    calculated_checksum = 0
    for char in data[1:]:  # Exclude the '$'
        calculated_checksum ^= ord(char)
    calculated_checksum = f"{calculated_checksum:02X}"
    return calculated_checksum == checksum.strip()


try:
    print("Waiting for data...")
    while True:
        line = ser.readline().decode('ascii', errors='ignore').strip()

        if line and line.startswith('$'):
            if validate_nmea_checksum(line):
                # Send the raw NMEA sentence to the Ubuntu system (client)
                client_socket.send(line.encode('utf-8'))
                print(f"Sent NMEA sentence: {line}")  # Debug log
            else:
                print(f"Invalid NMEA Sentence: {line}")

        time.sleep(0.1)  # Prevent CPU overuse

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    # Close connections
    client_socket.close()
    server_socket.close()
    ser.close()
    print("Serial port and server closed.")