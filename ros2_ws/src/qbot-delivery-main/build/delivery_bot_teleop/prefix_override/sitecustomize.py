import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nvidia/ENGR857_Salgado_Eriberto-main/ros2_ws/src/qbot-delivery-main/install/delivery_bot_teleop'
