import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nocc/Desktop/3dof_rehab_arm/install/Visualize'
