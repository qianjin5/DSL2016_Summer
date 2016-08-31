import fractions
import numpy as np
import random as random
import matplotlib.pyplot as plt
from R2rpy import R2rpy

from desiredstate import desiredstate
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

class NN_test(object):
    
    def init(self):
        pass
        
    
    def error_calculator(self, state1, state2):
        return np.mean(np.square(np.subtract(state1[0:3], state2[0:3])))
    
    def Test(self, agent = None, online = True, randomized_trajectory = False, given_trajectory = None, show = False, amp = 0.5, delay = 0.03, rf_int = 1, period = 15, traj_num = 0):

        # Interface-------------------------------------------------------------
        sim_time = 15

        outgoing_delay = param['outgoing_delay']
        incoming_delay = param['incoming_delay']

        # Set the initial commands and state
        # cmd: [roll, pitch, yaw_rate, z_vel]
        cmd_vel_init = np.array([0, 0, 0, 0])
        
        time_step = 0.015
        # state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')
        state_init = desiredstate(0, amp, period, traj_num)#np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0]) 

        pre_state = state_init
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
        record_states = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 20])
        record_desired = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 20])
        record_ref = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 20])
        record_cmds = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 5])
        record_inner = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 5])
        record_forces = np.zeros([sim_time / (time_per_tick * plot_div) + 1, 5])
    
        
        # Initialize error variables
        error = 0
        count = 0
        times = 0
        
        # Offline Only
        pre_desstate = state_init
        ref_state = state_init
        
        # Randomized Trajectory
        pre_d = state_init
        acc = (0, 0, 0)
        
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
                
                if randomized_trajectory == False:
                    des_state = desiredstate(tick / 1e6 + delay, amp, period, traj_num, pre_state, time_step)
                else:
                    error += self.error_calculator(cur_state, pre_d)
                    count += 1
                    if given_trajectory != None:
                        des_state = given_trajectory[count + 50]
                    else:
                        if count % 100 == 0:
                            a_max = 1 
                            acc_x = (random.random() - 0.5) * a_max
                            acc_y = (random.random() - 0.5) * a_max
                            acc_z = (random.random() - 0.5) * a_max
                            acc = (acc_x, acc_y, acc_z)
                        des_state = generate_desiredstate(pre_d, 0.015, acc, pre_state, time_step)
                    pre_d = des_state
                
                    
                # Get reference states from NN
                
                x, y, z, x_p, y_p, z_p, R11, R12, R13, R21, R22, R23, R31, R32, R33, p, q, r, z_pp = des_state
                
                if online:
                    x_c, y_c, z_c, x_p_c, y_p_c, z_p_c, R11_c, R12_c, R13_c, R21_c, R22_c, R23_c, R31_c, R32_c, R33_c, p_c, q_c, r_c, z_pp_c = cur_state
                else:
                    x_c, y_c, z_c, x_p_c, y_p_c, z_p_c, R11_c, R12_c, R13_c, R21_c, R22_c, R23_c, R31_c, R32_c, R33_c, p_c, q_c, r_c, z_pp_c = pre_desstate
                    pre_desstate = des_state
                    
                # Get reference states from NN        
                roll_c, pitch_c, yaw_c = R2rpy(np.array([R11_c, R12_c, R13_c, R21_c, R22_c, R23_c, R31_c, R32_c, R33_c]))
                roll, pitch, yaw = R2rpy(np.array([R11, R12, R13, R21, R22, R23, R31, R32, R33]))
                
                #des_state2 = desiredstate(tick / 1e6 + delay / 3.0, amp, period, traj_num)
                #x2, y2, z2, x_p2, y_p2, z_p2, R112, R122, R132, R212, R222, R232, R312, R322, R332, p2, q2, r2, z_pp2 = des_state2
                
                #roll2, pitch2, yaw2 = R2rpy(np.array([R112, R122, R132, R212, R222, R232, R312, R322, R332]))
                
                
                #des_state3 = desiredstate(tick / 1e6 + delay / 3.0 * 2.0, amp, period, traj_num)
                #x3, y3, z3, x_p3, y_p3, z_p3, R113, R123, R133, R213, R223, R233, R313, R323, R333, p3, q3, r3, z_pp3 = des_state3
                
                #roll3, pitch3, yaw3 = R2rpy(np.array([R113, R123, R133, R213, R223, R233, R313, R323, R333]))
                
                
                #inp = np.array([x - x_c, y - y_c, z - z_c, x2 - x_c, y2 - y_c, z2 - z_c, x_p2, y_p2, z_p2, x3 - x_c, y3 - y_c, z3 - z_c, x_p3, y_p3, z_p3, x_p_c, y_p_c, z_p_c, roll_c, pitch_c, yaw_c, p_c, q_c, r_c, z_pp_c, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp])
                inp = np.array([x - x_c, y - y_c, z - z_c, x_p_c, y_p_c, z_p_c, roll_c, pitch_c, yaw_c, p_c, q_c, r_c, z_pp_c, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp])
                
                #print R2rpy(np.array([R11_c, R12, R13_c, R21_c, R22_c, R23_c, R31_c, R32_c, R33_c]))
                
                if agent != None:
                    if times % (rf_int * 2) == 0:
                        ref_state = agent.getRef(inp)
                        
                        ref_state[0] += x
                        ref_state[1] += x_p
                        ref_state[2] += y
                        ref_state[3] += y_p
                        ref_state[4] += z
                        ref_state[5] += z_p
                        x, x_p, y, y_p, z, z_p = ref_state
                        #x, y, z = (ref_state[3] + x, ref_state[4] + y, ref_state[5] + z)
                        #gx, gy, gz = (ref_state[0] + x, ref_state[1] + y, ref_state[2] + z)
                    else:
                        x, y, z, x_p, y_p, z_p = ref_state[0], ref_state[1], ref_state[2], ref_state[3], ref_state[4], ref_state[5]
                    ref_state = x, y, z, x_p, y_p, z_p, R11, R12, R13, R21, R22, R23, R31, R32, R33, p, q, r, z_pp    
                    
                    times += 1
                else:
                    if randomized_trajectory == False:
                        ref_state = desiredstate(tick / 1e6, amp, period, traj_num, pre_state, time_step) 
                    else:
                        ref_state = x, y, z, x_p, y_p, z_p, R11, R12, R13, R21, R22, R23, R31, R32, R33, p, q, r, z_pp
                if randomized_trajectory == False:
                    des_state_plot = desiredstate(tick / 1e6, amp, period, traj_num, pre_state, time_step) # for plotting
                else:
                    des_state_plot = des_state
                ref_state_plot = ref_state # for plotting
                
                # Run desired state and current through DSLcontroller
                cmd_vel = DSLcontroller(cur_state, ref_state, param);
                
                pre_state = des_state
                
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
                des_state = desiredstate(tick / 1e6, amp, period, traj_num)
               
                if randomized_trajectory == False:
                    # Calculate error
                    error += self.error_calculator(cur_state, des_state)
                    count += 1
                
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
                #print "tick: ", tick
                #print "c state: ", cur_state
                #print "d state: ", des_state_plot
                #print "r state: ", ref_state_plot
                record_states[int(tick / (time_per_tick * plot_div))][:] = np.append(tick, cur_state)
                record_desired[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, des_state_plot)
                record_ref[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, ref_state_plot)
                record_cmds[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, cmd_vel)
                record_inner[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, inner_cmd)
                record_forces[int(tick / (time_per_tick * plot_div))][:]  = np.append(tick, forces_cmd)
        
        # Display error
        error /= count
        print "error: ", error
                

        # PLot------------------------------------------------------------------
        if show:
            # PLot------------------------------------------------------------------

            fig = plt.figure()
            
            '''
            td = fig.add_subplot(121, projection='3d')

            td.plot(np.array([record_states[i][1] for i in range(len(record_states))]),
                np.array([record_states[i][2] for i in range(len(record_states))]),
                    np.array([record_states[i][3] for i in range(len(record_states))]), 'b',label = 'Simulated Trajectory')
            td.plot(np.array([record_desired[i][1] for i in range(len(record_desired))]),
                np.array([record_desired[i][2] for i in range(len(record_desired))]),
                    np.array([record_desired[i][3] for i in range(len(record_desired))]), 'r',label = 'Desired Trajectory')
            td.plot(np.array([record_desired[i][1] for i in range(len(record_ref))]),
                np.array([record_ref[i][2] for i in range(len(record_ref))]),
                    np.array([record_ref[i][3] for i in range(len(record_ref))]), 'g',label = 'Reference Trajectory')
            td.set_xlabel('X (m)')
            td.set_ylabel('Y (m)')
            td.set_zlabel('Z (m)')
            plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
            plt.ticklabel_format(style='sci', axis='z', scilimits=(0,0))
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)
            '''
            '''
            xd = fig.add_subplot(311)
            xd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
                np.array([record_states[i][1] for i in range(len(record_states))]), 'b',label = 'Simulated X-Position')
            xd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
                np.array([record_desired[i][1] for i in range(len(record_desired))]), 'r',label = 'Desired X-Position')
            xd.plot(np.array([record_ref[i][0] for i in range(len(record_ref))]) / 1e6,
                np.array([record_ref[i][1] for i in range(len(record_ref))]), 'g--',label = 'Reference X-Position')
            xd.set_xlabel('Time (s)')
            xd.set_ylabel('Position (m)')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)

            yd = fig.add_subplot(312)
            yd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
                np.array([record_states[i][2] for i in range(len(record_states))]), 'b',label = 'Simulated Y-Position')
            yd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
                np.array([record_desired[i][2] for i in range(len(record_desired))]), 'r',label = 'Desired Y-Position')
            yd.plot(np.array([record_ref[i][0] for i in range(len(record_ref))]) / 1e6,
                np.array([record_ref[i][2] for i in range(len(record_ref))]), 'g--',label = 'Reference Y-Position')
            yd.set_xlabel('Time (s)')
            yd.set_ylabel('Position (m)')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)

            zd = fig.add_subplot(313)
            zd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
                np.array([record_states[i][3] for i in range(len(record_states))]), 'b',label = 'Simulated Z-Position')
            zd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
                np.array([record_desired[i][3] for i in range(len(record_desired))]), 'r',label = 'Desired Z-Position')
            zd.plot(np.array([record_ref[i][0] for i in range(len(record_ref))]) / 1e6,
                np.array([record_ref[i][3] for i in range(len(record_ref))]), 'g--',label = 'Reference Z-Position')
            zd.set_xlabel('Time (s)')
            zd.set_ylabel('Position (m)')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)
            '''
            td = fig.add_subplot(111, projection='3d')

            td.plot(np.array([record_states[i][4] for i in range(len(record_states))]) / 1e6,
                np.array([record_states[i][5] for i in range(len(record_states))]),
                    np.array([record_states[i][6] for i in range(len(record_states))]) / 1e6, 'b',label = 'Simulated Trajectory')
            td.plot(np.array([record_desired[i][4] for i in range(len(record_desired))]) / 1e6,
                np.array([record_desired[i][5] for i in range(len(record_desired))]),
                    np.array([record_desired[i][6] for i in range(len(record_desired))]) / 1e6, 'r',label = 'Desired Trajectory')
            td.plot(np.array([record_desired[i][4] for i in range(len(record_ref))]) / 1e6,
                np.array([record_ref[i][5] for i in range(len(record_ref))]),
                    np.array([record_ref[i][6] for i in range(len(record_ref))]) / 1e6, 'g',label = 'Reference Trajectory')
            td.set_xlabel('vX')
            td.set_ylabel('vY')
            td.set_zlabel('vZ')
            plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
            plt.ticklabel_format(style='sci', axis='z', scilimits=(0,0))
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=2, mode="expand", borderaxespad=0.)
            
            '''xd = fig.add_subplot(334)
            xd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
                np.array([record_states[i][4] for i in range(len(record_states))]), 'b',label = 'Simulated X-Velocity')
            xd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
                np.array([record_desired[i][4] for i in range(len(record_desired))]), 'r',label = 'Desired X-Velocity')
            xd.plot(np.array([record_ref[i][0] for i in range(len(record_ref))]) / 1e6,
                np.array([record_ref[i][4] for i in range(len(record_ref))]), 'g',label = 'Reference X-Velocity')
            xd.set_xlabel('Time (s)')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)

            yd = fig.add_subplot(335)
            yd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
                np.array([record_states[i][5] for i in range(len(record_states))]), 'b',label = 'Simulated Y-Velocity')
            yd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
                np.array([record_desired[i][5] for i in range(len(record_desired))]), 'r',label = 'Desired Y-Velocity')
            yd.plot(np.array([record_ref[i][0] for i in range(len(record_ref))]) / 1e6,
                np.array([record_ref[i][5] for i in range(len(record_ref))]), 'g',label = 'Reference Y-Velocity')
            yd.set_xlabel('Time (s)')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)

            zd = fig.add_subplot(336)
            zd.plot(np.array([record_states[i][0] for i in range(len(record_states))]) / 1e6,
                np.array([record_states[i][6] for i in range(len(record_states))]), 'b',label = 'Simulated Z-Velocity')
            zd.plot(np.array([record_desired[i][0] for i in range(len(record_desired))]) / 1e6,
                np.array([record_desired[i][6] for i in range(len(record_desired))]), 'r',label = 'Desired Z-Velocity')
            zd.plot(np.array([record_ref[i][0] for i in range(len(record_ref))]) / 1e6,
                np.array([record_ref[i][6] for i in range(len(record_ref))]), 'g',label = 'Reference Z-Velocity')
            zd.set_xlabel('Time (s)')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)
            
            '''
            '''xyd = fig.add_subplot(122)
            xyd.plot(np.array([record_states[i][1] for i in range(len(record_states))]),
                np.array([record_states[i][2] for i in range(len(record_states))]), 'b',label = 'Simulated Trajectory')
            xyd.plot(np.array([record_desired[i][1] for i in range(len(record_desired))]),
                np.array([record_desired[i][2] for i in range(len(record_desired))]), 'r',label = 'Desired Trajectory')
            xyd.plot(np.array([record_ref[i][1] for i in range(len(record_ref))]),
                np.array([record_ref[i][2] for i in range(len(record_ref))]), 'g',label = 'Reference Trajectory')
            xyd.set_xlabel('X (m)')
            xyd.set_ylabel('Y (m)')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)
            '''
            '''
            xyd = fig.add_subplot(338)
            xyd.plot(np.array([record_states[i][4] for i in range(len(record_states))]),
                np.array([record_states[i][5] for i in range(len(record_states))]), 'b',label = 'Simulated XY-Velocity')
            xyd.plot(np.array([record_desired[i][4] for i in range(len(record_desired))]),
                np.array([record_desired[i][5] for i in range(len(record_desired))]), 'r',label = 'Desired XY-Velocity')
            xyd.plot(np.array([record_ref[i][4] for i in range(len(record_ref))]),
                np.array([record_ref[i][5] for i in range(len(record_ref))]), 'g',label = 'Reference XY-Velocity')
            xyd.set_xlabel('vX (m)')
            xyd.set_xlabel('vY (m)')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                       ncol=1, mode="expand", borderaxespad=0.)
            '''
            #plt.tight_layout()
            plt.show()
        
        return error

