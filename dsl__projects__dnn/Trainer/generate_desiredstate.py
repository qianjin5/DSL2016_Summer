def generate_desiredstate(pre_state, time_step, acc):
    import math
    import random
    import numpy as np
    # Defines a desired vertical sinusoidal trajectory
    # state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')
    acc_x, acc_y, acc_z = acc
    # Define sinusoidal parameters
    '''z_init = 0 # initial z position (m)
    z_amp = 2 # amplitude of sinusoidal motion (m)
    period = 15.0 # sinusoidal period (s)
    period2 = 15.0
    phase_shift = 0.0 # phase shift (radians)
    '''
    # Define state variables
    des_state = np.zeros(19)
    # x, y, z
    des_state[0] = pre_state[0] + (pre_state[3] + acc_x * time_step / 2.0) * time_step
    des_state[1] = pre_state[1] + (pre_state[4] + acc_y * time_step / 2.0) * time_step
    des_state[2] = pre_state[2] + (pre_state[5] + acc_z * time_step / 2.0) * time_step
    # x', y', z'
    des_state[3] = pre_state[3] + acc_x * time_step
    des_state[4] = pre_state[4] + acc_y * time_step
    des_state[5] = pre_state[5] + acc_z * time_step
    
    '''t = 2 * math.pi * time / period + phase_shift
    a = 1.0
    
    x = a * (math.cos(t) ** 3.0)
    y = a * (math.sin(t) ** 3.0)
    z = a * math.sin(t)
    
    vx = -(2 * math.pi / period) * 3.0 * a * (math.cos(t) ** 2.0) * math.sin(t)
    vy = (2 * math.pi / period) * 3.0 * a * (math.sin(t) ** 2.0) * math.cos(t)
    vz = (2 * math.pi / period) * a * math.cos(t)
    
    des_state[0] = x
    des_state[1] = y
    des_state[2] = z
    
    des_state[3] = vx
    des_state[4] = vy
    des_state[5] = vz
    '''
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
