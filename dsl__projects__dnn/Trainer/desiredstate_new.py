def desiredstate_new(time, traj_num, traj):
    import math
    import numpy as np
    import random as random
    # Defines a desired vertical sinusoidal trajectory
    # state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')

    # Define sinusoidal parameters
    z_init = 0 # initial z position (m)
    z_amp = 1.0 # amplitude of sinusoidal motion (m)
    period = 7.5 # sinusoidal period (s)
    period2 = 7.5
    phase_shift = 0.0 # phase shift (radians)

    # Define state variables
    des_state = np.zeros(19)
    
    
    if traj_num == 0:
    
        # x, y, z
        des_state[0] = z_init + z_amp * math.cos(2 * math.pi * time / period + phase_shift) - z_amp
        des_state[1] = z_init + z_amp * math.sin(2 * math.pi * time / period + phase_shift)
        des_state[2] = z_init + z_amp * math.sin(2 * math.pi * time / period2 + phase_shift) + 1.2
        # x', y', z'
        des_state[3] = -(2 * math.pi / period) * z_amp * math.sin(2 * math.pi * time / period + phase_shift)
        des_state[4] = (2 * math.pi / period) * z_amp * math.cos(2 * math.pi * time / period + phase_shift)
        des_state[5] = (2 * math.pi / period2) * z_amp * math.cos(2 * math.pi * time / period2 + phase_shift)
    
    if traj_num == 1:
        t = 2 * math.pi * time / p
        a = amp
        
        x = a * (math.cos(t) ** 3.0)
        y = a * (math.sin(t) ** 3.0)
        z = a * math.sin(t)
        
        vx = -(2 * math.pi / p) * 3.0 * a * (math.cos(t) ** 2.0) * math.sin(t)
        vy = (2 * math.pi / p) * 3.0 * a * (math.sin(t) ** 2.0) * math.cos(t)
        vz = (2 * math.pi / p) * a * math.cos(t)
        
        des_state[0] = x
        des_state[1] = y
        des_state[2] = z
        
        des_state[3] = vx
        des_state[4] = vy
        des_state[5] = vz
    
    if traj_num == 2:    
    
        t = 2 * math.pi * time / p
        
        a = amp
        b = amp / 4.0
        c = amp / 5.0
        
        des_state[0] = (a-b) * math.cos(t) + c * math.cos((a/b - 1.0) * t)
        des_state[1] =  (a-b) * math.sin(t) + c * math.sin((a/b - 1.0) * t)
        des_state[2] = (a-b) * math.sin(t) + c * math.sin((a/b - 1.0) * t)
        
        des_state[3] = (-(a-b) * math.sin(t) - c * (a/b - 1.0) * math.sin((a/b - 1.0) * t)) * (2 * math.pi / p)
        des_state[4] = ((a-b) * math.cos(t) + c * (a/b - 1.0) * math.cos((a/b - 1.0) * t)) * (2 * math.pi / p)
        des_state[5] = ((a-b) * math.cos(t) + c * (a/b - 1.0) * math.cos((a/b - 1.0) * t)) * (2 * math.pi / p)

    
    if traj_num == 4 and traj != None:
		
		index = int(time / 0.015)
		#print index
		if index >= len(traj):
			index = len(traj) - 1
		x, y, z, vx, vy, vz = traj[index]
		des_state[0] = x
		des_state[1] = y
		des_state[2] = z
		des_state[3] = vx
		des_state[4] = vy
		des_state[5] = vz
    
    # R11 - R33
    des_state[6] = 1
    des_state[7] = 0
    des_state[8] = 0
    
    des_state[9] = 0
    des_state[10] = 1
    des_state[11] = 0
    
    des_state[12] = 0
    des_state[13] = 0
    des_state[14] = 1
    # p, q, r
    des_state[15] = 0
    des_state[16] = 0
    des_state[17] = 0
    # z''
    des_state[18] = 0

    return des_state
