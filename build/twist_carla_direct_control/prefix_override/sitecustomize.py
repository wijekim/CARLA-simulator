import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/airlab/ros2_ws/install/twist_carla_direct_control'
