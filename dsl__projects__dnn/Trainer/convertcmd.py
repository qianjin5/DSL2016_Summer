def convertcmd(outer_cmd, cur_state, param):
    import math
    import numpy as np
    from clamp import clamp
    # This function converts the roll, pitch, yaw_rate and z_dot commanded
    # through the cmd_vel topic to the p, q, r and thrust commands used by the
    # inner controller.
    
    # r commanded = yaw_rate commanded
    r_cmd = outer_cmd[2]
    
    # Desired z acceleration is (1/tau_z)*(z_dot_des - z_dot)
    z_ddot_des = (1.0 / param['tau_Iz']) * (outer_cmd[3] - cur_state[5])

    # Commanded thrust is (g + z_ddot_des)/R33
    thrust_cmd = (param['g'] + z_ddot_des) / cur_state[14]
    
    # Calculate the desired R13 and R23
    pitch_cur = math.asin(clamp(-cur_state[12], -1.0, 1.0))
    yaw_cur = math.acos(clamp(cur_state[6] / math.cos(pitch_cur), -1.0, 1.0))
    yaw_des = outer_cmd[2] * param['tau_Iyaw'] + yaw_cur
    R13_des = math.sin(yaw_des) * math.sin(outer_cmd[0]) + math.cos(yaw_des) * math.cos(outer_cmd[0]) * math.sin(outer_cmd[1])
    R23_des = math.cos(outer_cmd[0]) * math.sin(yaw_des) * math.sin(outer_cmd[1]) - math.cos(yaw_des) * math.sin(outer_cmd[0])

    # p_cmd = (R21*(R13_des-R13) - R11*(R23_des-R23))/(R33*tau_rp)
    p_cmd = (cur_state[9] * (R13_des-cur_state[8]) - cur_state[6] * (R23_des-cur_state[11])) / (cur_state[14] * param['tau_rp'])

    # q_cmd = (R22*(R13_des-R13) - R12*(R23_des-R23))/(R33*tau_rp)
    q_cmd = (cur_state[10] * (R13_des-cur_state[8]) - cur_state[7] * (R23_des-cur_state[11])) / (cur_state[14] * param['tau_rp'])

    # Construct inner command vector
    inner_cmd = np.array([p_cmd,  q_cmd, r_cmd, thrust_cmd])
    
    return inner_cmd
