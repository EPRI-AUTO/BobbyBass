import serial
import time

class RobotControl:
    def __init__(self, device, speed):
        """
        Initialize the RobotControl class with a serial connection to the robot.

        Args:
            device (str): The serial device (e.g., '/dev/ttyUSB0').
            speed (int): The baud rate for the serial connection (e.g., 9600).
        """
        try:
            # Open the serial connection
            self.ser = serial.Serial(device, speed, timeout=1)
            time.sleep(1)  # Wait for the connection to stabilize
        except serial.SerialException as e:
            # Handle errors if the serial connection fails
            print(f"Failed to open serial port: {e}")
            raise

    def close(self):
        """
        Close the serial connection cleanly.
        """
        if self.ser.is_open:
            self.ser.close()

    def battery_check(self):
        """
        Check the battery level by sending a command and reading the response.

        Returns:
            int: The battery level as a reconstructed integer from high and low bytes.
        """
        self.ser.write(b"(")  # Send the command to request battery level
        while self.ser.in_waiting < 2:
            time.sleep(0.001)  # Wait until at least 2 bytes are available

        if self.ser.in_waiting >= 2:
            highByte = self.ser.read(1)  # Read the high byte
            lowByte = self.ser.read(1)   # Read the low byte

            # Convert bytes to integers
            highByte = highByte[0]  # Extract the value from the byte object
            lowByte = lowByte[0]    # Extract the value from the byte object

            # Reconstruct the integer from the high and low bytes
            batval = (highByte << 8) | lowByte
            return batval

    def left_motor(self, direction, speed):
        """
        Control the left motor with a specified direction and speed.

        Args:
            direction (str): The direction of the motor ("FORWARD", "BACKWARD", "STOP", "QUICK STOP").
            speed (int): The speed level (1-8).
        """
        if direction == "FORWARD":
            self.ser.write(b"a")
        elif direction == "BACKWARD":
            self.ser.write(b"b")
        elif direction == "STOP":
            self.ser.write(b"c")
        elif direction == "QUICK STOP":
            self.ser.write(b"d")

        if speed == 1:
            self.ser.write(b"e")
        elif speed == 2:
            self.ser.write(b"f")
        elif speed == 3:
            self.ser.write(b"g")
        elif speed == 4:
            self.ser.write(b"h")
        elif speed == 5:
            self.ser.write(b"i")
        elif speed == 6:
            self.ser.write(b"j")
        elif speed == 7:
            self.ser.write(b"k")
        elif speed == 8:
            self.ser.write(b"l")

    def right_motor(self, direction, speed):
        """
        Control the right motor with a specified direction and speed.

        Args:
            direction (str): The direction of the motor ("FORWARD", "BACKWARD", "STOP", "QUICK STOP").
            speed (int): The speed level (1-8).
        """
        if direction == "FORWARD":
            self.ser.write(b"m")
        elif direction == "BACKWARD":
            self.ser.write(b"n")
        elif direction == "STOP":
            self.ser.write(b"o")
        elif direction == "QUICK STOP":
            self.ser.write(b"p")

        if speed == 1:
            self.ser.write(b"q")
        elif speed == 2:
            self.ser.write(b"r")
        elif speed == 3:
            self.ser.write(b"s")
        elif speed == 4:
            self.ser.write(b"t")
        elif speed == 5:
            self.ser.write(b"u")
        elif speed == 6:
            self.ser.write(b"v")
        elif speed == 7:
            self.ser.write(b"w")
        elif speed == 8:
            self.ser.write(b"x")

    def ind_light(self, pos, status):
        """
        Control an individual light at a specified position.

        Args:
            pos (int): The position of the light (1-6).
            status (str): The status of the light ("ON" or "OFF").
        """
        if pos == 1:
            if status == "ON":
                self.ser.write(b"y")
            elif status == "OFF":
                self.ser.write(b"z")
        elif pos == 2:
            if status == "ON":
                self.ser.write(b"0")
            elif status == "OFF":
                self.ser.write(b"1")
        elif pos == 3:
            if status == "ON":
                self.ser.write(b"2")
            elif status == "OFF":
                self.ser.write(b"3")
        elif pos == 4:
            if status == "ON":
                self.ser.write(b"4")
            elif status == "OFF":
                self.ser.write(b"5")
        elif pos == 5:
            if status == "ON":
                self.ser.write(b"6")
            elif status == "OFF":
                self.ser.write(b"7")
        elif pos == 6:
            if status == "ON":
                self.ser.write(b"8")
            elif status == "OFF":
                self.ser.write(b"9")

    def bump_sensor(self, num):
        """
        Check the status of a bump sensor.

        Args:
            num (int): The sensor number (1-8).

        Returns:
            str: "c" if the sensor is triggered, "o" otherwise.
        """
        if num == 1:
            self.ser.write(b"!")
        elif num == 2:
            self.ser.write(b"@")
        elif num == 3:
            self.ser.write(b"#")
        elif num == 4:
            self.ser.write(b"$")
        elif num == 5:
            self.ser.write(b"%")
        elif num == 6:
            self.ser.write(b"^")
        elif num == 7:
            self.ser.write(b"&")
        elif num == 8:
            self.ser.write(b"*")

        if self.ser.read() == b"c":
            return "c"
        else:
            return "o"
