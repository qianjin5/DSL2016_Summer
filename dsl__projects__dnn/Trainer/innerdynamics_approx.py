def innerdynamics_approx(old_state,forces,time,param):
    import numpy as np
    from determine_drag import determine_drag
    # This function takes in the current state values, the forces
    # applied by the motors, the quadrotor parameters and the time
    # to integrate over, and solves for the new state

    # NEW = OLD + delta_t*OLD_DERIV

    new_state = np.zeros(19)

    # Translational Velocities

    new_state[0] = old_state[0] + time*(old_state[3]) # x' = x'
    new_state[1] = old_state[1] + time*(old_state[4]) # y' = y'
    new_state[2] = old_state[2] + time*(old_state[5]) # z' = z'

    # Get the calculated drag force in x, y and z directions
    drag = determine_drag(old_state, param)

    # Translational Accelerations
    new_state[3] = old_state[3] + time*(sum(forces)*old_state[8] - drag[0])/param['m']                # x'' = (f*R13 - fx)/m
    new_state[4] = old_state[4] + time*(sum(forces)*old_state[11] - drag[1])/param['m']               # y'' = (f*R23 - fy)/m
    new_state[5] = old_state[5] + time*((sum(forces)*old_state[14] - drag[2])/param['m'] - param['g'])  # z'' = (f*R33 - fz)/m - g

    # Change in Rotation Matrix
    new_state[6] = old_state[6] + time*(old_state[7]*old_state[17] - old_state[8]*old_state[16]) # R11' = R12*r - R13*q
    new_state[7] = old_state[7] + time*(old_state[8]*old_state[15] - old_state[6]*old_state[17]) # R12' = R13*p - R11*r
    new_state[8] = old_state[8] + time*(old_state[6]*old_state[16] - old_state[7]*old_state[15]) # R13' = R11*q - R12*p
    new_state[9] = old_state[9] + time*(old_state[10]*old_state[17] - old_state[11]*old_state[16]) # R21' = R22*r - R23*q
    new_state[10] = old_state[10] + time*(old_state[11]*old_state[15] - old_state[9]*old_state[17]) # R22' = R23*p - R21*r
    new_state[11] = old_state[11] + time*(old_state[9]*old_state[16] - old_state[10]*old_state[15]) # R23' = R21*q - R22*p
    new_state[12] = old_state[12] + time*(old_state[13]*old_state[17]- old_state[14]*old_state[16]) # R31' = R32*r - R33*q
    new_state[13] = old_state[13] + time*(old_state[14]*old_state[15] - old_state[12]*old_state[17]) # R32' = R33*p - R31*r
    new_state[14] = old_state[14] + time*(old_state[12]*old_state[16] - old_state[13]*old_state[15]) # R33' = R31*q - R32*p

    # p' = (1/Ix)*(L*(f2-f4) + (Iy-Iz)*r*q)
    new_state[15] = old_state[15] + time*((1/param['Ix'])*(param['L']*(forces[1]-forces[3]) + (param['Iy']-param['Iz'])*old_state[16]*old_state[17]))

    # q' = (1/Iy)*(L*(f3-f1) + (Iz-Ix)*r*p)
    new_state[16] = old_state[16] + time*((1/param['Iy'])*(param['L']*(forces[2]-forces[0]) + (param['Iz']-param['Ix'])*old_state[15]*old_state[17]))

    # r' = (1/Iz)*(K*(f1-f2+f3-f4) + (Ix-Iy)*p*q)
    new_state[17] = old_state[17] + time*((1/param['Iz'])*(param['K']*(forces[0]-forces[1]+forces[2]-forces[3]) + (param['Ix']-param['Iy'])*old_state[15]*old_state[16]))

    # z'' = (f*R33 - fz)/m - g
    new_state[18] = (sum(forces)*old_state[14] - drag[2])/param['m'] - param['g']

    return new_state
