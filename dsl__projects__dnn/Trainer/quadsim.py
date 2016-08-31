import fractions
import numpy as np
import matplotlib.pyplot as plt

from desiredstate import desiredstate
from determineforces import determineforces
from DSLcontroller import DSLcontroller
from parameters import param
from clamp import clamp
from convertcmd import convertcmd 
from innerdynamics_approx import innerdynamics_approx

from mpl_toolkits.mplot3d import Axes3D
from parameters import param

# Interface-------------------------------------------------------------
sim_time = 20

outgoing_delay = param['outgoing_delay']
incoming_delay = param['incoming_delay']

# Set the initial commands and state
# cmd: [roll, pitch, yaw_rate, z_vel]
cmd_vel_init = np.array([0, 0, 0, 0])

# state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')
state_init = np.array([0, 0, 1.5, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0])

# Take a sample every plot_div ticks for plotting purposes
plot_div = 50

# Run the simulator
sim_time = sim_time * 1000000

# End Interface---------------------------------------------------------

# Set the rates of the inner and outer control loops
inner_loop_cycle = 1000 # update rate of inner loop (us)
outer_loop_cycle = 15000 # outer loop update rate (us)

# minimize the number of ticks
time_per_tick = fractions.gcd(
    fractions.gcd(inner_loop_cycle, outer_loop_cycle), 
    fractions.gcd(outgoing_delay, incoming_delay))

# Initialize the commands and state
# state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')
cmd_vel = cmd_vel_init
cur_state = state_init

# Initialize delay buffers
outgoing_buffer = np.array([np.append([0], cmd_vel_init)])
incoming_buffer = np.array([np.append([0], state_init)])

# Initialize data recording vectors
record_states = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 20])
record_desired = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 20])
record_cmds = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 5])
record_inner = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 5])
record_forces = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 5])

for tick in np.linspace(0, sim_time, (sim_time / time_per_tick + 1)):
    # increment delay times and update buffers
    n = len(outgoing_buffer)
    for i in range(n):
	outgoing_buffer[i][0] += time_per_tick
        
    if n > 1:
        delay_index = 1
        for i in range(2, n):
            if outgoing_buffer[i][0] >= outgoing_delay:
                delay_index = i
        outgoing_buffer = outgoing_buffer[delay_index::]
    
    n = len(incoming_buffer)
    for i in range(n):
	incoming_buffer[i][0] += time_per_tick
    if (n > 1):
        delay_index = 1
        for i in range(2, n):
            if incoming_buffer[i][0] >= incoming_delay:
                delay_index = i;
        incoming_buffer = incoming_buffer[delay_index::];
    
    
    # Outer Loop
    if tick % outer_loop_cycle == 0:
        
        # Get current and desired state for this time
        cur_state = incoming_buffer[0][1:]
        des_state = desiredstate(tick / 1e6)
        des_state_plot = des_state # for plotting
        

        # Run desired state and current through DSLcontroller
        cmd_vel = DSLcontroller(cur_state, des_state, param);
        
        #fudge factor amplifies the roll and pitch command
        #to make the simulation results look realistic to real drone
        #performance. For an ideal simulation application, if necessary,
        #the user should disable this factor by setting param(32) to 1.
        cmd_vel[0] *= param['fudge_factor']
        cmd_vel[1] *= param['fudge_factor']
        
        #clamp the pitch and roll angle between maxmium euler angle, in
        #real life, this step takes place in the ARDrone Bridge
        cmd_vel[0] = clamp(cmd_vel[0], -param['max_euler_angle'], param['max_euler_angle'])
        cmd_vel[1] = clamp(cmd_vel[1], -param['max_euler_angle'], param['max_euler_angle'])
        
        # Add to delay buffer        
        outgoing_buffer = np.concatenate((outgoing_buffer, np.array([np.insert(cmd_vel, 0, 0)])), axis = 0)
    
    # Inner Loop
    if tick % inner_loop_cycle == 0:
        
        # Obtain latest command and state
        cmd_vel = outgoing_buffer[0][1:]
        cur_state = incoming_buffer[0][1:]
       
        # Convert latest outer commands to inner commands
        inner_cmd = convertcmd(cmd_vel,cur_state,param)
        
        #Run the inner inner loop which tracks the rotation rate
        for i in range(param['inner_oicycle_ratio']):
           # Have onboard controller determine the commanded forces
           forces_cmd = determineforces(cur_state, inner_cmd, param)
           # Compute the new state from the innerdynamics
           cur_state = innerdynamics_approx(cur_state, forces_cmd, inner_loop_cycle/1e6/param['inner_oicycle_ratio'],param)
        
        #Update the cur_state in the buffer
        
        incoming_buffer = np.concatenate((incoming_buffer, np.array([np.insert(cur_state, 0, 0)])),axis = 0)
    
    
    # Record for plotting later
    if (tick / time_per_tick) % plot_div == 0:
        record_states[int(tick / (time_per_tick * plot_div))][:] = np.append(tick, cur_state)
        record_desired[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, des_state_plot)
        record_cmds[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, cmd_vel)
        record_inner[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, inner_cmd)
        record_forces[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, forces_cmd)
    
        

# PLot------------------------------------------------------------------

fig = plt.figure()

td = fig.add_subplot(221, projection='3d')

td.plot(np.array([record_states[i][1] for i in range(len(record_states))]) / 1e6,
	np.array([record_states[i][2] for i in range(len(record_states))]),
        np.array([record_states[i][3] for i in range(len(record_states))]) / 1e6, 'b',label = 'Simulated Trajectory')
td.plot(np.array([record_desired[i][1] for i in range(len(record_desired))]) / 1e6,
	np.array([record_desired[i][2] for i in range(len(record_desired))]),
        np.array([record_desired[i][3] for i in range(len(record_desired))]) / 1e6, 'r',label = 'Desired Trajectory')
td.set_xlabel('X')
td.set_ylabel('Y')
td.set_zlabel('Z')
plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
plt.ticklabel_format(style='sci', axis='z', scilimits=(0,0))
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)

xd = fig.add_subplot(222)
xd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
	np.array([record_states[i][1] for i in range(len(record_states))]), 'b',label = 'Simulated X-Position')
xd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
	np.array([record_desired[i][1] for i in range(len(record_desired))]), 'r',label = 'Desired X-Position')
xd.set_xlabel('Time (s)')
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)

yd = fig.add_subplot(223)
yd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
	np.array([record_states[i][2] for i in range(len(record_states))]), 'b',label = 'Simulated Y-Position')
yd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
	np.array([record_desired[i][2] for i in range(len(record_desired))]), 'r',label = 'Desired Y-Position')
yd.set_xlabel('Time (s)')
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)

zd = fig.add_subplot(224)
zd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
	np.array([record_states[i][3] for i in range(len(record_states))]), 'b',label = 'Simulated Z-Position')
zd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
	np.array([record_desired[i][3] for i in range(len(record_desired))]), 'r',label = 'Desired Z-Position')
zd.set_xlabel('Time (s)')
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)

plt.tight_layout()
plt.show()



