import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rovteam/Desktop/Melvin/drone_cage_control-main/install/utility_objects'
