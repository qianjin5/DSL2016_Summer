import fractions
import numpy as np
import matplotlib.pyplot as plt

import random as random
import math

from desiredstate import desiredstate
from determineforces import determineforces
from DSLcontroller import DSLcontroller
from parameters import param
from clamp import clamp
from convertcmd import convertcmd 
from innerdynamics_approx import innerdynamics_approx

from mpl_toolkits.mplot3d import Axes3D
from parameters import param

from R2rpy import R2rpy
from rpy2R import rpy2R

class NN_generate_data2(object):
    
    def init(self):
        pass
        
    def random_state(self, time, amp, period1, period2, period3):
        # Define state variables
        z_init = 0
        z_amp = (int(time / 36.0) + 1) * amp
        phase_shift = 0
        des_state = np.zeros(19)
        # x, y, z
        des_state[0] = z_init + z_amp * math.cos(2 * math.pi * time / period1 + phase_shift) - z_amp
        des_state[1] = z_init + z_amp * math.cos(2 * math.pi * time / period2 + phase_shift) - z_amp
        des_state[2] = z_init + z_amp * math.cos(2 * math.pi * time / period3 + phase_shift) - z_amp
        # x', y', z'
        des_state[3] = -(2 * math.pi / period1) * z_amp * math.sin(2 * math.pi * time / period1 + phase_shift)
        des_state[4] = -(2 * math.pi / period2) * z_amp * math.sin(2 * math.pi * time / period2 + phase_shift)
        des_state[5] = -(2 * math.pi / period3) * z_amp * math.sin(2 * math.pi * time / period3 + phase_shift)
        # R11 - R33
        des_state[6] = 1
        des_state[7] = 0
        des_state[8] = 0
        
        des_state[9] = 0
        des_state[10] = 1
        des_state[11] = 0
        
        des_state[12] = 0
        des_state[13] = 0
        des_state[14] = 1
        # p, q, r
        des_state[15] = 0
        des_state[16] = 0
        des_state[17] = 0
        # z''
        des_state[18] = 0
        
        return des_state
    
    def generate_data(self, periods, amp, show = True):

        # Interface-------------------------------------------------------------
        sim_time = 240
        count = 0
        threshold = 12
        target_count = 15
        time_step = 0.15
        data = []
        states = []
        
        outgoing_delay = param['outgoing_delay']
        incoming_delay = param['incoming_delay']

        # Set the initial commands and state
        # cmd: [roll, pitch, yaw_rate, z_vel]
        cmd_vel_init = np.array([0, 0, 0, 0])

        # state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')
        state_init = self.random_state(time_step, amp, periods[0], periods[1], periods[2])
        desire_state = self.random_state(time_step, amp, periods[0], periods[1], periods[2])
        
        # Take a sample every plot_div ticks for plotting purposes
        plot_div = 1

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
                
                state = incoming_buffer[0][1:]
                if count >= threshold:
                    x1, y1, z1, x_p1, y_p1, z_p1, R111, R121, R131, R211, R221, R231, R311, R321, R331, p1, q1, r1, z_pp1 = states[count - threshold]
                    x2, y2, z2, x_p2, y_p2, z_p2, R112, R122, R132, R212, R222, R232, R312, R322, R332, p2, q2, r2, z_pp2 = state
                    x, y, z, x_p, y_p, z_p, R11, R12, R13, R21, R22, R23, R31, R32, R33, p, q, r, z_pp = desire_state
                    xm, ym, zm, x_pm, y_pm, z_pm, R11m, R12m, R13m, R21m, R22m, R23m, R31m, R32m, R33m, pm, qm, rm, z_ppm = states[count - threshold / 3 * 2]
                    xm2, ym2, zm2, x_pm2, y_pm2, z_pm2, R11m2, R12m2, R13m2, R21m2, R22m2, R23m2, R31m2, R32m2, R33m2, pm2, qm2, rm2, z_ppm2 = states[count - threshold / 3]
                    #data.append((np.array([x - x2, x_p - x_p2, y - y2, y_p - y_p2, z - z2, z_p - z_p2, xm - x2, x_pm - x_p2, ym - y2, y_pm - y_p2, zm - z2, z_pm - z_p2]), 
                    #np.array([x1, y1, z1, x_p1, y_p1, z_p1, R111, R121, R131, R211, R221, R231, R311, R321, R331, p1, q1, r1, z_pp1, 
                    #x2, y2, z2, x_p2, y_p2, z_p2, R112, R122, R132, R212, R222, R232, R312, R322, R332, p2, q2, r2, z_pp2])))
                    data.append((np.array([x - x2, x_p - x_p2, y - y2, y_p - y_p2, z - z2, z_p - z_p2]), 
                    np.array([x1, y1, z1, x_p1, y_p1, z_p1, R111, R121, R131, R211, R221, R231, R311, R321, R331, p1, q1, r1, z_pp1, 
                    xm, ym, zm, x_pm, y_pm, z_pm, R11m, R12m, R13m, R21m, R22m, R23m, R31m, R32m, R33m, pm, qm, rm, z_ppm,
                    xm2, ym2, zm2, x_pm2, y_pm2, z_pm2, R11m2, R12m2, R13m2, R21m2, R22m2, R23m2, R31m2, R32m2, R33m2, pm2, qm2, rm2, z_ppm2,
                    x2, y2, z2, x_p2, y_p2, z_p2, R112, R122, R132, R212, R222, R232, R312, R322, R332, p2, q2, r2, z_pp2])))
                
                if count == target_count:
                    count = 0
                    states = []
                    desire_state = self.random_state(tick / 1e6, amp, periods[0], periods[1], periods[2])
                    
                states.append(state)
                count += 1
                
                # Get current and desired state for this time
                cur_state = incoming_buffer[0][1:]
                des_state = desire_state
                des_state_plot = des_state # for plotting
                
                # Get reference states from NN
                '''
                des_state, cur_state
                '''
                
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
        
           
        if show:
           self.plot(record_states, record_desired)
                
        return data
    
    def plot(self, record_states, record_desired):
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

