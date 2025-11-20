import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jinwoo/webots_ros2_ws/install/webots_ros2_turtlebot'
