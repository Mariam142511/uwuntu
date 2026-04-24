import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yestlisanchez/uwuntu/week_3/puzzlebot_sim/install/puzzlebot_sim'
