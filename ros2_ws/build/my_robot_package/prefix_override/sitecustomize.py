import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gio2/Robotics/freshStart/Gazebo-Ros2-Demo/ros2_ws/install/my_robot_package'
