import fractions
import numpy as np
import random as random
import matplotlib.pyplot as plt
from R2rpy import R2rpy

from desiredstate_new import desiredstate_new
from generate_desiredstate import generate_desiredstate
from determineforces import determineforces
from DSLcontroller import DSLcontroller
from parameters import param
from clamp import clamp
from convertcmd import convertcmd 
from innerdynamics_approx import innerdynamics_approx

from mpl_toolkits.mplot3d import Axes3D
from parameters import param

from learning_agent import LearningAgent

class TrajTestor(object):
    
    def init(self):
        pass
        
    
    def error_calculator(self, state1, state2):
        return np.mean(np.square(np.subtract(state1[0:3], state2[0:3])))
    
    def Test(self, agent = None, traj_num = 0, traj = None, init_s = None, delay = 0):

        # Interface-------------------------------------------------------------
        sim_time = 15
        
        if traj != None:
			sim_time = 0.015 * len(traj)

        outgoing_delay = param['outgoing_delay']
        incoming_delay = param['incoming_delay']
        # Set the initial commands and state
        # cmd: [roll, pitch, yaw_rate, z_vel]
        cmd_vel_init = np.array([0, 0, 0, 0])
        
        # state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')
        if init_s == None:
            state_init = desiredstate_new(0, traj_num, traj) #np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0]) 
        else:
            state_init = init_s
        # Take a sample every plot_div ticks for plotting purposes
        plot_div = 50

        # Run the simulator
        sim_time = sim_time * 1000000

        # End Interface---------------------------------------------------------

        # Set the rates of the inner and outer control loops
        inner_loop_cycle = param['inner_loop_cycle'] # update rate of inner loop (us)
        outer_loop_cycle = param['outer_loop_cycle'] # outer loop update rate (us)

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
        record_states = []
        record_desired = []
        record_ref = []
        record_cmds = []
        record_inner = []
        record_forces = []
        
        # Initialize error variables
        error = 0
        count = 0
        times = 0
        
        print delay
        for tick in np.linspace(0, sim_time, ((sim_time) / time_per_tick + 1)):
            if tick / 1000000.0 < delay: continue
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
            
            des_state = desiredstate_new(tick / 1e6, traj_num, traj)
            
            # Outer Loop
            if tick % outer_loop_cycle == 0:    
                # Get current and desired state for this time
                cur_state = incoming_buffer[0][1:]
                
                des_state = desiredstate_new(tick / 1e6, traj_num, traj)
                
                
                x, y, z, x_p, y_p, z_p, R11, R12, R13, R21, R22, R23, R31, R32, R33, p, q, r, z_pp = des_state
                x_c, y_c, z_c, x_p_c, y_p_c, z_p_c, R11_c, R12_c, R13_c, R21_c, R22_c, R23_c, R31_c, R32_c, R33_c, p_c, q_c, r_c, z_pp_c = cur_state
                
                if agent == None:
					ref_state = des_state
                else:
					roll_c, pitch_c, yaw_c = R2rpy(np.array([R11_c, R12_c, R13_c, R21_c, R22_c, R23_c, R31_c, R32_c, R33_c]))
					roll, pitch, yaw = R2rpy(np.array([R11, R12, R13, R21, R22, R23, R31, R32, R33]))
					inp = np.array([x - x_c, y - y_c, z - z_c, x_p_c, y_p_c, z_p_c, roll_c, pitch_c, yaw_c, p_c, q_c, r_c, z_pp_c, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp])
					ref = agent.getRef(inp)
					ref[0] += x
					ref[1] += x_p
					ref[2] += y
					ref[3] += y_p
					ref[4] += z
					ref[5] += z_p
					x, x_p, y, y_p, z, z_p = ref
					ref_state = x, y, z, x_p, y_p, z_p, R11, R12, R13, R21, R22, R23, R31, R32, R33, p, q, r, z_pp
					
                
                error += (((x - x_c) ** 2 + (y - y_c) ** 2 + (z - z_c) ** 2) ** 0.5)
                count += 1
                
                # Run desired state and current through DSLcontroller
                cmd_vel = DSLcontroller(cur_state, ref_state, param);
                
                #fudge factor amplifies the roll and pitch command
                #to make the simulation results look realistic to real drone
                #performance. For an ideal simulation application, if necessary,
                #the user should disable this factor by setting param(32) to 1.
                cmd_vel[0] *= param['fudge_factor_r']
                cmd_vel[1] *= param['fudge_factor_p']
                
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
                des_state = desiredstate_new(tick / 1e6, traj_num, traj)
               
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
            record_states.append(np.append(tick, cur_state))
            record_desired.append(np.append(tick, des_state))
            #record_cmds.append(np.append(tick, cmd_vel))
            #record_inner.append(np.append(tick, inner_cmd))
            #record_forces.append(np.append(tick, forces_cmd))
        
        # Display error
        error /= count
        print "error: ", error
        
        return record_states, record_desired

