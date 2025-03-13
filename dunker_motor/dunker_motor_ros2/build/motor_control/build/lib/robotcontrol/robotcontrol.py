import serial
import time


class RobotControl:
    def __init__(self, device, speed):
        self.ser = serial.Serial(device, speed, timeout=1)
        time.sleep(1) 
        
        return

    def left_motor(self, direction, speed):
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

        return

    def right_motor(self, direction, speed):
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

        return

    def ind_light(self, pos, status):
        if pos == 1:
            if status == "ON":
                self.ser.write(b"y")
            elif status == "OFF":
                self.ser.write(b"z")
        if pos == 2:
            if status == "ON":
                self.ser.write(b"0")
            elif status == "OFF":
                self.ser.write(b"1")
        if pos == 3:
            if status == "ON":
                self.ser.write(b"2")
            elif status == "OFF":
                self.ser.write(b"3")
        if pos == 4:
            if status == "ON":
                self.ser.write(b"4")
            elif status == "OFF":
                self.ser.write(b"5")
        if pos == 5:
            if status == "ON":
                self.ser.write(b"6")
            elif status == "OFF":
                self.ser.write(b"7")
        if pos == 6:
            if status == "ON":
                self.ser.write(b"8")
            elif status == "OFF":
                self.ser.write(b"9")

        return

    def bump_sensor(self, num):
        if num == 1:
            self.ser.write(b"!")
        if num == 2:
            self.ser.write(b"@")
        if num == 3:
            self.ser.write(b"#")
        if num == 4:
            self.ser.write(b"$")
        if num == 5:
            self.ser.write(b"%")
        if num == 6:
            self.ser.write(b"^")
        if num == 7:
            self.ser.write(b"&")
        if num == 8:
            self.ser.write(b"*")

        if self.ser.read() == b"c":
            return 1
        else:
            return 0

        return
