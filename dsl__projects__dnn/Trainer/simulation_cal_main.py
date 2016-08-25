from TrajTestor import TrajTestor
from NN_generate_data import NN_generate_data
from NN_generate_data2 import NN_generate_data2
from learning_agent import LearningAgent
from generate_randomized_trajectory import generate_randomized_trajectory
import numpy as np
import matplotlib.pyplot as plt
from parameters import param
from R2rpy import R2rpy
from rpy2R import rpy2R

def save_to_file(file_name, data):
    np.save(file_name, data)

def load_from_file(file_name):
    try:
        return np.load(file_name + ".npy")
    except:
        return None

t_ros_t = []
p_x_ros_t = []
p_y_ros_t = []
p_z_ros_t = []
t_ros_t2 = []
p_x_ros_t2 = []
p_y_ros_t2 = []
p_z_ros_t2 = []
t_sim_t = []
p_x_sim_t = []
p_y_sim_t = []
p_z_sim_t = []


fig = plt.figure()
x1 = fig.add_subplot(221)
x2 = fig.add_subplot(222)
x3 = fig.add_subplot(223)
x4 = fig.add_subplot(224)

num01 = 1
num02 = 1

for i in range(num01):
	traj_ros = load_from_file("trajectory_rosreal_x" + str(i))

	#print traj_ros

	t_ros_t.append([s[0] for s in traj_ros])
	p_x_ros_t.append([s[1] for s in traj_ros])
	p_y_ros_t.append([s[2] for s in traj_ros])
	p_z_ros_t.append([s[3] for s in traj_ros])

	print len(traj_ros)
	
for i in range(num01):
	x1.plot(t_ros_t[i], p_x_ros_t[i], "r-")
	x2.plot(t_ros_t[i], p_y_ros_t[i], "r-")
	x3.plot(t_ros_t[i], p_z_ros_t[i], "r-")
	x4.plot(p_x_ros_t[i], p_y_ros_t[i], "r-")

first_state = traj_ros[200][1:]
delay = traj_ros[200][0]

'''
for i in range(num02):
	traj_ros = load_from_file("aft_trajectory" + str(i))

	#print traj_ros

	t_ros_t2.append([s[0] for s in traj_ros])
	p_x_ros_t2.append([s[1] for s in traj_ros])
	p_y_ros_t2.append([s[2] for s in traj_ros])
	p_z_ros_t2.append([s[3] for s in traj_ros])

	print len(traj_ros)
	
for i in range(num02):
	x1.plot(t_ros_t2[i], p_x_ros_t2[i], "g-")
	x2.plot(t_ros_t2[i], p_y_ros_t2[i], "g-")
	x3.plot(t_ros_t2[i], p_z_ros_t2[i], "g-")
	x4.plot(p_x_ros_t2[i], p_y_ros_t2[i], "g-")
'''
num03 = 1
for i in range(num03):
	traj_ros = load_from_file("trajectory_rossimx" + str(i))

	#print traj_ros

	t_sim_t.append([s[0] for s in traj_ros])
	p_x_sim_t.append([s[1] for s in traj_ros])
	p_y_sim_t.append([s[2] for s in traj_ros])
	p_z_sim_t.append([s[3] for s in traj_ros])

	print len(traj_ros)
	
for i in range(num03):
		
	
	x1.plot(t_sim_t[i], p_x_sim_t[i], "b-")

	x2.plot(t_sim_t[i], p_y_sim_t[i], "b-")

	x3.plot(t_sim_t[i], p_z_sim_t[i], "b-")

	x4.plot(p_x_sim_t[i], p_y_sim_t[i], "b-")

plt.show()


print delay
print first_state
x_c, y_c, z_c, x_p_c, y_p_c, z_p_c, roll_c, pitch_c, yaw_c, p_c, q_c, r_c, z_pp_c = first_state
R11_c, R12_c, R13_c, R21_c, R22_c, R23_c, R31_c, R32_c, R33_c = rpy2R(np.array([roll_c, pitch_c, yaw_c]))
init_state = np.array([x_c, y_c, z_c, x_p_c, y_p_c, z_p_c, R11_c, R12_c, R13_c, R21_c, R22_c, R23_c, R31_c, R32_c, R33_c, p_c, q_c, r_c, z_pp_c])                
#init_state = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0])        

print init_state

testor = TrajTestor()

'''
traj = load_from_file("trajectory")

t = [i * 0.015 for i in range(len(traj))]
p_x = [s[0] for s in traj]
p_y = [s[1] for s in traj]
p_z = [s[2] for s in traj]
'''

agent = LearningAgent()

t = []
a_x = []
a_y = []
a_z = []
r_x = []
r_y = []
r_z = []

num = 1


for i in range(num):
	#param["fudge_factor"] = 3.125+0.6*i
	#a_traj, r_traj = testor.Test(None, 4, traj, init_state)
	a_traj, r_traj = testor.Test(None, 0, None, init_state, delay)
	print a_traj[0][0]
	t.append([s[0] / 1e6 for s in a_traj])
	a_x.append([s[1] for s in a_traj])
	a_y.append([s[2] for s in a_traj])
	a_z.append([s[3] for s in a_traj])
	r_x.append([s[1] for s in r_traj])
	r_y.append([s[2] for s in r_traj])
	r_z.append([s[3] for s in r_traj])
	#print param["fudge_factor"]

num2 = num03
fig = plt.figure()
x = fig.add_subplot(221)
#x.plot(t_ros, p_x_ros, "r-")
for j in range(num2):
	x.plot(t_ros_t[j], p_x_ros_t[j], "r-")
	x.plot(t_sim_t[j], p_x_sim_t[j], "y-")
for i in range(num):
    x.plot(t[i], a_x[i], "b-")
    x.plot(t[i], r_x[i], "g--")

x = fig.add_subplot(222)
#x.plot(t_ros, p_y_ros, "r-")
for j in range(num2):
	x.plot(t_ros_t[j], p_y_ros_t[j], "r-")
	x.plot(t_sim_t[j], p_y_sim_t[j], "y-")
for i in range(num):
    x.plot(t[i], a_y[i], "b-")
    x.plot(t[i], r_y[i], "g--")

x = fig.add_subplot(223)
#x.plot(t_ros, p_z_ros, "r-")
for j in range(num2):
	x.plot(t_ros_t[j], p_z_ros_t[j], "r-")
	x.plot(t_sim_t[j], p_z_sim_t[j], "y-")
for i in range(num):
    x.plot(t[i], a_z[i], "b-")
    x.plot(t[i], r_z[i], "g--")

x = fig.add_subplot(224)
#x.plot(p_x_ros, p_z_ros, "r-")
for j in range(num2):
	x.plot(p_x_ros_t[j], p_y_ros_t[j], "r-")
	x.plot(p_x_sim_t[j], p_y_sim_t[j], "y-")
for i in range(num):
    x.plot(a_x[i], a_y[i], "b-")
    x.plot(r_x[i], r_y[i], "g--")

plt.show()

'''
testor = TrajTestor()

traj = load_from_file("trajectory")

t = [i * 0.015 for i in range(len(traj))]
p_x = [s[0] for s in traj]
p_y = [s[1] for s in traj]
p_z = [s[2] for s in traj]


agent = LearningAgent()

t = []
a_x = []
a_y = []
a_z = []
r_x = []
r_y = []
r_z = []

num = 1

for i in range(num):
	param["outgoing_delay"] = i * 25000
	a_traj, r_traj = testor.Test(None, 4, traj)
	#a_traj, r_traj = testor.Test(None, 0)
	t.append([s[0] / 1e6 for s in a_traj])
	a_x.append([s[1] for s in a_traj])
	a_y.append([s[2] for s in a_traj])
	a_z.append([s[3] for s in a_traj])
	r_x.append([s[1] for s in r_traj])
	r_y.append([s[2] for s in r_traj])
	r_z.append([s[3] for s in r_traj])
	print param["outgoing_delay"]

fig = plt.figure()
x = fig.add_subplot(221)
for i in range(num):
	x.plot(t[i], a_x[i], "b-")
	x.plot(t[i], r_x[i], "g--")

x = fig.add_subplot(222)
for i in range(num):
	x.plot(t[i], a_y[i], "b-")
	x.plot(t[i], r_y[i], "g--")

x = fig.add_subplot(223)
for i in range(num):
	x.plot(t[i], a_z[i], "b-")
	x.plot(t[i], r_z[i], "g--")

x = fig.add_subplot(224)
for i in range(num):
	x.plot(a_x[i], a_z[i], "b-")
	x.plot(r_x[i], r_z[i], "g--")

plt.show()
'''
