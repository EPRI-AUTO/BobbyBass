from robotcontrol import RobotControl
import time
import pygame

robot = RobotControl("/dev/ttyACM0", 115200)

pygame.joystick.init()
pygame.init()
joystick = pygame.joystick.Joystick(0)

run = True
while run:
	robot.ind_light(2, "ON")
	pygame.event.pump()

	if joystick.get_button(2):
		robot.stuck("OFF")

	if joystick.get_button(0):
		boost = 2
	else:
		boost = 1
	# Joystick input
	move = joystick.get_axis(1)
	turn = joystick.get_axis(2)

	if abs(move) < 0.1:
		move = 0
	if abs(turn) < 0.1:
		turn = 0

	leftSpeed = -move + turn
	rightSpeed = -move - turn

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

	if abs(leftSpeed) > 0.95:
		left_speed = 4
	elif abs(leftSpeed) > 0.5:
		left_speed = 3
	elif abs(leftSpeed) > 0.25:
		left_speed = 2
	else:
		left_speed = 1
	if abs(rightSpeed) > 0.95:
		right_speed = 4
	elif abs(rightSpeed) > 0.5:
		right_speed = 3
	elif abs(rightSpeed) > 0.25:
		right_speed = 2
	else:
		right_speed = 1

	# D-Pad inputs
	if joystick.get_hat(0) == (1, 1):
		left_dir = right_dir = "FORWARD"
		left_speed = 3
		right_speed = 1
	elif joystick.get_hat(0) == (-1, 1):
		left_dir = right_dir = "FORWARD"
		left_speed = 1
		right_speed = 3
	elif joystick.get_hat(0) == (0, 1) and joystick.get_button(0):
		boost = 1
		left_dir = right_dir = "FORWARD"
		left_speed = right_speed = 6
	elif joystick.get_hat(0) == (0, 1) and joystick.get_button(3):
		boost = 1
		left_dir = right_dir = "FORWARD"
		left_speed = right_speed = 8
	elif joystick.get_hat(0) == (0, 1):
		left_dir = right_dir = "FORWARD"
		left_speed = right_speed = 3
	elif joystick.get_hat(0) == (0, -1):
		left_dir = right_dir = "BACKWARD"
		left_speed = right_speed = 2
	elif joystick.get_hat(0) == (-1, 0):
		left_dir = "BACKWARD"
		right_dir = "FORWARD"
		left_speed = right_speed = 1
	elif joystick.get_hat(0) == (1, 0):
		left_dir = "FORWARD"
		right_dir = "BACKWARD"
		left_speed = right_speed = 1

	robot.left_motor(left_dir, left_speed*boost)
	robot.right_motor(right_dir, right_speed*boost)
	
	if joystick.get_button(1):
	    run = False
	    
pygame.quit()
robot.left_motor("STOP", 1)
robot.right_motor("STOP", 1)
robot.ind_light(2, "OFF")
