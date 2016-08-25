serverAddr = 'localhost'
serverAddrFull = 'http://' + serverAddr 


# for Python MysQL Connector
configInfo = {
			'user': 'DSLPath_guest',
			'password': '',
			'host': serverAddr,
			'database': 'DSLPath_db'}

import os
curpath = os.path.abspath(os.curdir)
trajinfopath = os.path.join(curpath, "exp_data/traj_info.json")
respathbase = os.path.join(curpath, "exp_data/trial_res")

# Hard code the physical range of the arena
import json
phys_range = json.dumps([[-2.0, 2.0], [0.0, 2.0]])
