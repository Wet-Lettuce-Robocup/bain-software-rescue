import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bain/Documents/GitHub/robocup-ros/ros/bain-software-rescue/install/rescue_state_bc'
