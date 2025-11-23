import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/stl/JNU/connectstudy/ros2_ws/install/indy_dcp_bridge'
