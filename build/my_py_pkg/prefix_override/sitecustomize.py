import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rharvey/code/ros2_for_beginners_2025/ros2_ws/install/my_py_pkg'
