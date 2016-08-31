def DSLcontroller(cur_state,des_state,param):
    import math
    import numpy as np
    from R2rpy import R2rpy
    from clamp import clamp
    # This function takes in the current state, desired state and set
    # of parameters, and applies the DSL controller equations to determine
    # the commands that would be sent to the drone over the cmd_vel topic.
    # state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')
    # cmd_vel: (roll_cmd, pitch_cmd, yaw_rate_cmd, z_dot_cmd)

    # z_dot_cmd = (1/tau_z)*(z_des - z_cur)
    z_dot_cmd =(2.0*param['eta_y']/param['tau_z'])*(des_state[5]-cur_state[5]) + (1.0/(param['tau_z']**2))*(des_state[2]-cur_state[2]) 

    # Determine the current and desired yaw values from R11 and R31  
    rpy_des = R2rpy(des_state[6:15])
    rpy_cur = R2rpy(cur_state[6:15])
    yaw_cur = rpy_cur[2]
    yaw_des = rpy_des[2]
    thrust = (param['g'] + cur_state[18])/(cur_state[14])

    # x_ddot_cmd = (2*eta/tau_x)*(x_dot_des - x_dot_cur) + (1/tau_x^2)*(x_des - x_cur)
    
    x_ddot_cmd = (2.0* param['eta_y']/param['tau_y'])*(des_state[3]-cur_state[3]) + (1.0/(param['tau_y']**2))*(des_state[0]-cur_state[0])
    
    # y_ddot_cmd = (2*eta/tau_y)*(y_dot_des - y_dot_cur) + (1/tau_y^2)*(y_des - y_cur)
    
    y_ddot_cmd = (2.0 *param['eta_y']/param['tau_y'])*(des_state[4]-cur_state[4]) + (1.0/(param['tau_y']**2))*(des_state[1]-cur_state[1])

    # Determine "roll" and "pitch" in the global frame
    roll_global_cmd = clamp(math.asin(clamp(y_ddot_cmd/thrust,-1,1)), -param['max_euler_angle'], param['max_euler_angle'])
    pitch_global_cmd = clamp(math.asin(clamp(x_ddot_cmd/(thrust*math.cos(roll_global_cmd)),-1,1)), -param['max_euler_angle'], param['max_euler_angle']);
    pitch_cmd = pitch_global_cmd*math.cos(yaw_cur) + roll_global_cmd*math.sin(yaw_cur)
    roll_cmd = - pitch_global_cmd*math.sin(yaw_cur) + roll_global_cmd*math.cos(yaw_cur)

    # Determine the commanded yaw rate
    yaw_rate_cmd = clamp((1/param['tau_yaw'])*(yaw_des-yaw_cur),-param['max_yaw_rate'], param['max_yaw_rate'])

    # Create output vector
    # The minus sign before roll_cmd is due to the sign convention roll angles
    # (Positive Roll causes acceleration in negative y-direction)
    cmd_vel = np.array([-roll_cmd, pitch_cmd, yaw_rate_cmd, z_dot_cmd])

    return cmd_vel
