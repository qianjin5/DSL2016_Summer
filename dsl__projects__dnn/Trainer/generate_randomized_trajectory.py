
from desiredstate import desiredstate
from generate_desiredstate import generate_desiredstate
import random

def generate_randomized_trajectory(a_max, time_step, sim_time):
    pre_d = desiredstate(0)
    acc = (0, 0, 0)
    des = []
    des.append(pre_d)
    for i in range(int(sim_time / time_step) + 100):
        if (i + 1) % 100 == 0:
            acc_x = (random.random() - 0.5) * a_max * 2.0
            acc_y = (random.random() - 0.5) * a_max * 2.0
            acc_z = (random.random() - 0.5) * a_max * 2.0
            acc = (acc_x, acc_y, acc_z)
        des_state = generate_desiredstate(pre_d, time_step, acc)
        des.append(des_state)
        pre_d = des_state
    print des
    return des
