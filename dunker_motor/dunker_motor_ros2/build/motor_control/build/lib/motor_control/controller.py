import pygame
import time
from robotcontrol import RobotControl

def main():

    robot = RobotControl("/dev/ttyACM0", 115200)

    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

    pygame.init()

    SCREEN_WIDTH = 500
    SCREEN_HEIGHT = 500

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    bump = [0, 0, 0, 0, 0, 0, 0, 0]
    run = True
    while run:
        # Joystick input
        hor = pygame.joystick.Joystick(0).get_axis(0)
        vert = pygame.joystick.Joystick(0).get_axis(1)

        if abs(hor) < 0.1 and abs(vert) < 0.1:
            leftSpeed = 0
            rightSpeed = 0
        else:
            if hor < 0:
                rightSpeed = -vert
                leftSpeed = rightSpeed * (1 - abs(hor))
            else:
                leftSpeed = -vert
                rightSpeed = leftSpeed * (1 - abs(hor))

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

        if abs(leftSpeed) > 0.75:
            left_speed = 4
        elif abs(leftSpeed) > 0.5:
            left_speed = 3
        elif abs(leftSpeed) > 0.25:
            left_speed = 2
        else:
            left_speed = 1
        if abs(rightSpeed) > 0.75:
            right_speed = 4
        elif abs(rightSpeed) > 0.5:
            right_speed = 3
        elif abs(rightSpeed) > 0.25:
            right_speed = 2
        else:
            right_speed = 1

        # A button input
        if pygame.joystick.Joystick(0).get_button(1):
            run = False

        # D-Pad inputs
        if pygame.joystick.Joystick(0).get_hat(0) == (0, 1):
            left_dir = right_dir = "FORWARD"
            left_speed = right_speed = 2
        if pygame.joystick.Joystick(0).get_hat(0) == (0, -1):
            left_dir = right_dir = "BACKWARD"
            left_speed = right_speed = 4
        if pygame.joystick.Joystick(0).get_hat(0) == (-1, 0):
            left_dir = "BACKWARD"
            right_dir = "FORWARD"
            left_speed = right_speed = 2
        if pygame.joystick.Joystick(0).get_hat(0) == (1, 0):
            left_dir = "FORWARD"
            right_dir = "BACKWARD"
            left_speed = right_speed = 2

        robot.left_motor(left_dir, left_speed)
        robot.right_motor(right_dir, right_speed)

        #Bump sensor detection
        for i in [1, 2, 3, 4, 5, 6, 7, 8]:
            bump[i-1] = robot.bump_sensor(i)
        print(bump)
        time.sleep(0.1)

        if sum(bump) > 0:
            robot.left_motor("QUICK STOP", 1)
            robot.right_motor("QUICK STOP", 1)

            if bump[0] == 1 or bump[1] == 1:
                bumpDir1 = "FORWARD"
                bumpDir2 = "LEFT"
            elif bump[2] == 1 or bump[3] == 1:
                bumpDir1 = "FORWARD"
                bumpDir2 = "RIGHT"
            elif bump[4] == 1 or bump[5] == 1:
                bumpDir1 = "BACKWARD"
                bumpDir2 = "LEFT"
            elif bump[6] == 1 or bump[7] == 1:
                bumpDir1 = "BACKWARD"
                bumpDir2 = "RIGHT"

            robot.left_motor(bumpDir1, 1)
            robot.right_motor(bumpDir1, 1)
            time.sleep(4)
            robot.left_motor("STOP", 1)
            robot.right_motor("STOP", 1)
            time.sleep(1)

            if bumpDir2 == "LEFT":
                robot.left_motor("BACKWARD", 1)
                robot.right_motor("FORWARD", 1)
            else:
                robot.left_motor("FORWARD", 1)
                robot.right_motor("BACKWARD", 1)

            time.sleep(2)
            robot.left_motor("STOP", 1)
            robot.right_motor("STOP", 1)
            time.sleep(2)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

    robot.left_motor("STOP", 1)
    robot.right_motor("STOP", 1)

    pygame.quit

if __name__ == '__main__':
    main()