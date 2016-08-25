def determine_drag(state, param):
    import numpy as np
    # This function calculates the drag force acting on the drone assuming a
    # linear model of the drag proportional to speed.

    # Construct the Rotation Matrix from current state
    R1 = state[6:9]
    R2 = state[9:12]
    R3 = state[12:15]
    R = np.array([R1, R2, R3])
    
    #Transform the velocity from world frame into body frame
    vel_b = np.dot(np.linalg.inv(R) , [state[3], state[4], state[5]])
    
    #Calculates the drag force in the body frame, according to the linear
    #model
    fp = param['airdrag_x'] * vel_b[0]
    fr = param['airdrag_y'] * vel_b[1]
    fy = param['airdrag_z'] * vel_b[2]
    
    #Transform the forces into world frame, in x, y, and z direction
    drag = np.dot(R, [fp, fr, fy])
    return drag
