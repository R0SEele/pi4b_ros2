import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rose/pi4b_ros2/car_ws/install/test_control_pkg'
