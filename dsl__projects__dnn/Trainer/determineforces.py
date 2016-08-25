def determineforces(cur_state, inner_cmd, param):
    import numpy as np
    from clamp import clamp
    # This function determines the equivalent commanded forces given commands
    # for thrust, p, q and r.


    # Determine A matrix: L = param(3), K = param(4), m = param(1), tau_p =
    # param(18), tau_q = param(19), tau_r = param(20), Ix = param(5), Iy =
    # param(6), Iz = param(7)
    A = [[0, param['L'], 0, -param['L']], 
	 [-param['L'], 0, param['L'], 0], 
	 [param['K'], -param['K'], param['K'], -param['K']], 
	 [(1.0/param['m']), (1.0/param['m']), (1.0/param['m']), (1.0/param['m'])]]
     
    # Determine b matrix
    J = np.diag([param['Ix'], param['Iy'], param['Iz']], 0)
    #J = np.diag([param['Ix'], param['Iy'], param['Iz']])
    
    # inertial matrix
    omega = [cur_state[15], cur_state[16], cur_state[17]] # angular velocity
    rate_vector = [(1.0 / param['tau_p']) * (inner_cmd[0] - cur_state[15]),
		   (1.0 / param['tau_q']) * (inner_cmd[1] - cur_state[16]), 
		   (1.0 / param['tau_r']) * (inner_cmd[2] - cur_state[17])]
    
    b = np.append(np.add(np.dot(J, rate_vector), np.cross(omega , np.dot(J, omega))),inner_cmd[3])
    #b = J.dot(rate_vector) + np.cross(omega, J.dot(omega))
    #b = np.concatenate(b, inner_cmd[3])
    
    # Solve for the forces
    #forces = np.dot(np.linalg.inv(A), b)
    forces = np.linalg.solve(A, b)
    
    # Noise
    std = param["noise_const"]
    forces[0] += np.random.normal(0, std, 1)
    forces[1] += np.random.normal(0, std, 1)
    forces[2] += np.random.normal(0, std, 1)
    forces[3] += np.random.normal(0, std, 1)
    
    # Clamp the force between maxmium and minimum values
    forces[0] = clamp(forces[0], param['fmin'], param['fmax'])
    forces[1] = clamp(forces[1], param['fmin'], param['fmax'])
    forces[2] = clamp(forces[2], param['fmin'], param['fmax'])
    forces[3] = clamp(forces[3], param['fmin'], param['fmax'])

    return forces
