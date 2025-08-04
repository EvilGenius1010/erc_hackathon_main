import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mahe/Desktop/hk/erc_hackathon_main/Automation/install/controller'
