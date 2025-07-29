import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/francesco/Desktop/tirocinio/ros2_ws/install/isaac_arm_ml_control'
