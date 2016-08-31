def rpy2R(rpy):
    import math
    import numpy as np
# Rotation takes the convention of Yaw-Pitch-Roll
# takes roll, pitch and yaw values: rpy = [roll, pitch, yaw]
# outputs the corresponding rotation matrix: R = [R11 R12 R13 ... R33];
	
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    R11 = math.cos(yaw) * math.cos(pitch)
    R12 = math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.cos(roll) * math.sin(yaw)
    R13 = math.sin(yaw) * math.sin(roll) + math.cos(yaw) * math.cos(roll) * math.sin(pitch)
    R21 = math.cos(pitch) * math.sin(yaw)
    R22 = math.cos(yaw) * math.cos(roll) + math.sin(roll) * math.sin(pitch) * math.sin(yaw)
    R23 = math.cos(roll) * math.sin(yaw) * math.sin(pitch) - math.cos(yaw) * math.sin(roll)
    R31 = -math.sin(pitch)
    R32 = math.cos(pitch) * math.sin(roll)
    R33 = math.cos(pitch) * math.cos(roll)

    R = np.array([R11, R12, R13, R21, R22, R23, R31, R32, R33])

    return R
