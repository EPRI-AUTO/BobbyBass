import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/epriauto/ros2_ws/dunker_motor/dunker_motor_ros2/install/motor_control'
