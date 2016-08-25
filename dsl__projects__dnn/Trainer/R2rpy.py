def R2rpy(R):
    import math
    import numpy as np
    # Takes a rotation matrix and determines the corresponding roll, pitch and
    # yaw
    # R = [R11 R12 R13 R21 R22 R23 R31 R32 R33]

    # pitch = asin(clamp(-R(7),-1,1));
    # roll = asin(clamp(R(8)/math.cos(pitch),-1,1));
    # yaw = asin(clamp(R(4)/math.cos(pitch),-1,1));
    roll = math.atan(R[7] / R[8])
    pitch = math.atan(-R[6] / math.sqrt(R[7] ** 2 + R[8] ** 2))
    yaw = math.atan((R[3] / R[0]))
    sin_yaw = R[3] / math.cos(pitch)
    
    if (sin_yaw > 0 and yaw < 0): 
	    yaw = yaw + math.pi
    elif (sin_yaw < 0 and yaw > 0):
	    yaw = yaw - math.pi
    rpy = np.array([roll, pitch, yaw])

    return rpy
