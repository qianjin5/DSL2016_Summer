def desiredstate(time, amp = 0.5, p = 15.0, traj_num = 0, pre_state = None, time_step = 0.18):
    import math
    import numpy as np
    import random as random
    # Defines a desired vertical sinusoidal trajectory
    # state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')

    # Define sinusoidal parameters
    z_init = 0 # initial z position (m)
    z_amp = amp # amplitude of sinusoidal motion (m)
    period = p # sinusoidal period (s)
    period2 = p
    phase_shift = 0.0 # phase shift (radians)

    # Define state variables
    des_state = np.zeros(19)
    
    if traj_num == 0:
    
        # x, y, z
        des_state[0] = z_init + z_amp * math.sin(2 * math.pi * time / period + phase_shift)
        des_state[1] = z_init + z_amp / 2.0 * math.cos(2 * math.pi * time / period + phase_shift)
        des_state[2] = z_init + z_amp / 2.0 * math.sin(2 * math.pi * time / period2 + phase_shift)
        # x', y', z'
        des_state[3] = (2 * math.pi / period) * z_amp * math.cos(2 * math.pi * time / period + phase_shift)
        des_state[4] = -(2 * math.pi / period) * z_amp / 2.0 * math.sin(2 * math.pi * time / period + phase_shift)
        des_state[5] = (2 * math.pi / period2) * z_amp / 2.0 * math.cos(2 * math.pi * time / period2 + phase_shift)
    
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
    
    if traj_num == 3:
        if pre_state == None:
            des_state[0] = 0
            des_state[1] = 0
            des_state[2] = 0
            des_state[3] = 0
            des_state[4] = 0
            des_state[5] = 0
        else:
            x = pre_state[0]
            y = pre_state[1]
            z = pre_state[2]
            vx = pre_state[3]
            vy = pre_state[4]
            vz = pre_state[5]
            ax = (random.random() - 0.5) * amp * 2.0
            ay = (random.random() - 0.5) * amp * 2.0
            az = (random.random() - 0.5) * amp * 2.0
            
            x+= (vx + ax * time_step / 2.0) * time_step
            y+= (vy + ay * time_step / 2.0) * time_step
            z+= (vz + az * time_step / 2.0) * time_step
            
            vx += ax * time_step
            vy += ay * time_step
            vz += az * time_step
            
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
