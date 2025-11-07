import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/csci_420_robotics_labs/lab6_ws/install/simple_control'
