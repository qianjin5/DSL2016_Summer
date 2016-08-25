#!/usr/bin/env python2
	
# Created by Jingxing Qian - Last update: Monday Aug.8, 2016
# Contact me: jingxing.qian@mail.utoronto.ca

# This is a publisher that emits a reference signal, calculated by neural network, based on the desired path and the drone's status.

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('dsl__projects__dnn')
import rospy
import numpy as np
import math

from std_msgs.msg import Empty, Int64MultiArray
from ardrone_autonomy.msg import Navdata
from dsl__utilities__ardrone import DroneStatus
from dsl__utilities__msg.msg import StateVector
from dsl__projects__dnn import Parameters
from dsl__projects__dnn.msg import StateData
from dsl__projects__dnn.msg import State
from dsl__projects__dnn.msg import MultiStateData

from learning_agent import LearningAgent

from demo_config import trajinfopath
import json

class CorrectionSignal:
    def __init__(self):
        
        # use deep neural network?
        self.use_DNN = False
        # use to reset timer
        self.can_change = True
        # set initial state as landed
        self.state = DroneStatus.Landed
        # feedback DNN or not, default True
        self.feedback = True
        
        # important states
        self.curr_state = StateVector()  # from Vicon
        self.desired_state = StateData() # from reference signal
        self.est_state = StateData()     # converted from Vicon, (self.curr_state)
        self.ref_state = StateData()     # for plotting
        self.dnn_ref_state = StateData() # followed by the quadrotor
        
        # History
        self.history_ref = []
        self.history_des = []

        
        # time when changes from takeoff to flying
        self.start_time = rospy.get_time()
        
        # global time after takeoff
        self.cur_time = rospy.get_time()
        
        # default flight time with head and tail
        self.duration = 15
        self.traj_start_time = 0.0
        self.traj_end_time = self.duration
        self.prev_time = 0
        
        # dnn calculates a reference point for every x outer loops
        self.DNN_delay = 10
        self.cumulative_error = 0.0
        self.count = 0
        self.delay_loop = 0
        self.dstate = [0,0,0,0,0,0] 
        
        # so the trajectory starts at where quadrotor changes from takeoff to flying
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
        
        # accumulating experience
        # record as fly, default False
        self.rec_Exp = False
        self.experience = self.load_experience()
        self.EXP_delay = 10
        self.pre_state = None
        
        # Publishers
        self.pub = rospy.Publisher('path_coordinates', StateData, queue_size=20)              # Reference trajectory the quadrotor actually follows
        self.pub_d = rospy.Publisher('desired_coordinates', StateData, queue_size=20)         # Desired trajectory specified by user
        self.pub_state = rospy.Publisher('full_state', State, queue_size=20)                  # Actual trajectory performed by quadrotor for plotting
        self.pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=10)
        #self.pub_sys_ready = rospy.Publisher('system_ready', Empty, queue_size = 1)
        
        # Subscriber
        self.sub_curr_state = rospy.Subscriber('estimated_state', StateVector, self.get_cur_state)             # get state from Vicon  
        self.sub_desired_state_adv = rospy.Subscriber('desired_coordinates_delay', MultiStateData, self.find_ref)   # get desired state from reference_signal.py
        self.sub_state = rospy.Subscriber('ardrone/navdata', Navdata, self.change_state)                       # is it flying?
        self.sub_takeoff = rospy.Subscriber('ardrone/takeoff', Empty, self.change_state)
        self.sub_land = rospy.Subscriber('ardrone/land', Empty, self.land_drone)
        self.sub_toggle = rospy.Subscriber('DNN_Toggle', Empty, self.toggle)                                   # get flight time for trajectory
        self.sub_newtime = rospy.Subscriber('newtime', Int64MultiArray, self.update_time)                                # if use user-defined trajectory, update flight duration
        self.sub_init_cond = rospy.Subscriber('init_cond', StateData, self.set_offset)                         # what's the desired position after taking off? shift the trajectory                                                                                               # in x-y so it aligns with your quadrotor's position after takeoff

        self.sub_Rec = rospy.Subscriber('rec_data', Empty, self.rec_data)                                      # record experience as we go?
        self.sub_Learn = rospy.Subscriber('trainDNN', Empty, self.train_data)              
        self.sub_dnn_feedback = rospy.Subscriber('dnn_feedback', Empty, self.feedback_toggle)                  # use feedback DNN or non_feedback?
        
        
    def change_state(self, navdata):
        if type(navdata) == Navdata:
            # reset everything here when taking off
            self.state = navdata.state
            if self.state == DroneStatus.Flying and self.can_change == True:
                self.start_time = rospy.get_time()
                self.can_change = False
        else:
            # they are here because real quadrotor's state may change between landing and flying during landing
            # so better reset at timeoff
            self.can_change = True 
            self.cumulative_error = 0.0	
            self.count = 0	
            
                
    def land_drone(self,msg):
        print 'count: ', self.count
        print 'exp: ', len(self.experience)
        
        # determine average error
        avgerr = str(self.cumulative_error / self.count)
        print '---Average Error: ' + avgerr + 'm---'
        
        # write into local file
        f = open(trajinfopath, "r")
        dictobj = json.loads(f.read())
        dictobj['avgerr'] = avgerr
        f.close()
        f = open(trajinfopath, "w")
        f.write(json.dumps(dictobj))
        f.close()
          	
        self.save_experience()             
            
    def update_time(self,new_time):
        # receive new flight time from reference_signal.py
        self.duration = new_time.data[0]
        self.traj_start_time = new_time.data[1]
        self.traj_end_time = new_time.data[2]
          
        print '---New Total Flight Time: ' + str(self.duration) + 's---' 
        print '---New Trajectory Starts at: ' + str(self.traj_start_time) + 's---' 
        print '---New Trajectory Finishes at: ' + str(self.traj_end_time) + 's---' 
            
            
    def set_offset(self, init_state):
        # set trajectory offset so traj starts at place where the quadrotor takeoff
        if self.state == DroneStatus.Landed:		
            self.x_offset = self.curr_state[0] - init_state.x		
            self.y_offset = self.curr_state[1] - init_state.y
            self.z_offset = 1.0 - init_state.z 
        
    def copy_state(self, target):
        # make a copy of StateData
        new = StateData()
        new.header, \
        new.x, new.y, new.z, \
        new.vx, new.vy, new.vz, \
        new.ax, new.ay, new.az, \
        new.roll, new.pitch, new.yaw = \
        target.header,  \
        target.x, target.y, target.z, \
        target.vx, target.vy, target.vz, \
        target.ax, target.ay, target.az, \
        target.roll, target.pitch, target.yaw
        return new
        
    def conv_13(self, target):
        # convert a StateData to the desired 13 bit state vector
        new_state = np.array([target.x, target.y, target.z,
                              target.vx, target.vy, target.vz,
                              0, 0, 0, 0, 0, 0, 0])		
        return new_state

    def get_cur_state(self, curr_state):	
        # conver a StateVector to the desired 13 bit state vector
        pos = list(curr_state.pos)
        vel = list(curr_state.vel)
        rpq = list(curr_state.euler)
        omega = list(curr_state.omega_b)
        acc = list(curr_state.acc)
        zpp = [acc[2]]
        self.curr_state = np.array(pos+vel+rpq+omega+zpp)
        #update current position for plotting
        self.est_state.x = pos[0]
        self.est_state.y = pos[1]
        self.est_state.z = pos[2]
        self.est_state.vx = vel[0]
        self.est_state.vy = vel[1]
        self.est_state.vz = vel[2]
        self.est_state.ax = acc[0]
        self.est_state.ay = acc[1]
        self.est_state.az = acc[2]
        self.est_state.roll = rpq[0]
        self.est_state.pitch = rpq[1]
        self.est_state.yaw = rpq[2]
        
    def toggle(self,msg):
        # is DNN on? press D on keyboard to switch
        self.use_DNN = not self.use_DNN
        if self.use_DNN == True:
            print('---DNN Enabled---')
        else:
            print('---DNN Disabled---')	
            
    def feedback_toggle(self,msg):
        # do we use feedback for DNN?
        self.feedback = not self.feedback
        if self.feedback == True:
            print('---Feedback Enabled---')
        else:
            print('---Feedback Disabled---')	        
            
    def rec_data(self,msg):
        # record-as-fly toggle function
        self.rec_Exp = not self.rec_Exp  
        if self.rec_Exp == True:
            print('---Experience Recording Enabled---')
        else:
            print('---Experience Recording Disabled---')
    
    def load_experience(self):
        # load past experience from ~/.ros
        try:
            f = np.load('Experience.npy')
            print '---Experience Loaded---'
            return np.ndarray.tolist(f)
        except: 
            print '---No Previous Experience Found---'
            return []
            
    def save_experience(self):
        # save experience when landing
        print '---Experience Saved---'
        np.save('Experience.npy', self.experience)        
            
    def train_data(self,msg):
        # train DNN, PLEASE USE THE TRAINER PROGRAM FOR BETTER PERFORMANCE!
        if len(self.experience) < 1000:
            print '---Insufficient Experience---'
            print '---Current Experience: ' + str(len(self.experience))+'/1000---'
            return
        if self.state == DroneStatus.Landed:
            print '---Training in Progress---'
            x,y,z,xv,yv,zv = [i for i in range(6)]
            processed_data = agent.process_data(self.experience)
            for i in range(1):
                for j in [x,y,z]:  	
                    agent.Train(processed_data, j, i*1000, False)
            print '---1000 Iterations Finished---' 
            agent.load_network(agent.sess)       
        else:
            print '---Please Land the Vechicle First---' 
           
    def find_ref(self, des_state):
        # use neural network to find reference trajectory
        
        # add the offset to trajectory 
        hist_size = des_state.size
        for i in range(hist_size):
            des_state.data[i].x += self.x_offset 
            des_state.data[i].y += self.y_offset 	
        
        # copy or convert states to 13 bit state vector for DNN
        self.ref_state = self.copy_state(des_state.data[0])
        self.desired_state = self.conv_13(des_state.data[0])
        self.est_state.header = des_state.data[0].header
               
        if self.state == DroneStatus.Flying:
            self.delay_loop += 1
            if self.use_DNN:	
                if self.delay_loop >= self.DNN_delay:

                    if self.feedback:
                        # for feedback dnn
                        temp = self.curr_state
                    else:
                        # for non-feedback dnn
                        temp = self.history_des[-1]  if len(self.history_des) != 0 else [0 for i in range(13)]
                    
                    # in case we use past states
                    if len(self.history_des) >= hist_size: 
                        data = np.array([])

                        ##################
                        # extract states from desired trajectory
                        states = [self.conv_13(des_state.data[i]) for i in range(hist_size)]
                        # append current desired state
                        data = np.append(data, temp[3:])
                        
                        for i in range(1,hist_size):
                            # append future desired states
                            data = np.append(data, states[i][0:3] - temp[0:3])
                            data = np.append(data, states[i][3:])
                        
                        ##################
                        
                        # find the difference between reference state and desired state	
                        self.dstate = agent.getRef(data)
                        
                    else:
                        self.dstate = [0,0,0,0,0,0]
                       
                    dx, dy, dz, dvx, dvy, dvz = self.dstate    
   
                    # add the calculated difference to the desired state to get reference state
                    self.dnn_ref_state = self.copy_state(des_state.data[0])
                    self.dnn_ref_state.x += dx 
                    self.dnn_ref_state.y += dy 
                    self.dnn_ref_state.z += dz 
		    
                    self.dnn_ref_state.vx += dvx   
                    self.dnn_ref_state.vy += dvy 
                    self.dnn_ref_state.vz += dvz 
                    
                    self.delay_loop = 0	
                    
                    ref_13 = self.conv_13(self.dnn_ref_state)
                    # in case we use historical observed states 
                    self.history_ref.append(ref_13[0:6])
                    # for non-feedback dnn currently
                    self.history_des.append(self.desired_state[:])
                    
                self.pub.publish(self.dnn_ref_state)    
                
                
            else:
                self.dstate = [0,0,0,0,0,0]
                # publish reference 
                self.pub.publish(des_state.data[0])
                # collecting data during regular fight for future training
                if self.rec_Exp and self.delay_loop >= self.EXP_delay: 
                    expstate = np.append(self.curr_state, self.desired_state[0:6])
                    self.experience.append(expstate)
                    self.delay_loop = 0	
                    
            # for plotting

            dx, dy, dz, dvx, dvy, dvz = self.dstate
            
            # add the calculated difference to the desired state for plotting
            self.ref_state = self.copy_state(des_state.data[0])
            self.ref_state.x += dx 
            self.ref_state.y += dy 
            self.ref_state.z += dz 
                
            self.ref_state.vx += dvx   
            self.ref_state.vy += dvy 
            self.ref_state.vz += dvz 
            
            
            # determine current flight time
            timer = self.cur_time - self.start_time
            
            #if float('{:05.2f}'.format(timer)) % 0.5 == 0.0:
            if (timer < 0.50):
                self.prev_time = 2*timer
            if (int(self.prev_time) != int(2*timer)):
                self.prev_time = 2*timer
                # print current flight time
                print '---Timer: ' + '{:04.1f}'.format(timer) + 's---'
            
            if self.traj_start_time < timer < self.traj_end_time:
                # determine cumulative error
                self.cumulative_error += np.linalg.norm(self.desired_state[0:3] - self.curr_state[0:3])
                self.count += 1
                
                # for plotting
                if self.count % self.DNN_delay == 0:
                    state = State()
                    state.t = timer
                    
                    state.des_x = des_state.data[0].x
                    state.des_y = des_state.data[0].y
                    state.des_z = des_state.data[0].z
            
                    state.ref_x = self.ref_state.x
                    state.ref_y = self.ref_state.y
                    state.ref_z = self.ref_state.z
            
                    state.est_x = self.est_state.x
                    state.est_y = self.est_state.y
                    state.est_z = self.est_state.z
                    
                    state.des_vx = des_state.data[0].vx
                    state.des_vy = des_state.data[0].vy
                    state.des_vz = des_state.data[0].vz
            
                    state.ref_vx = self.ref_state.vx
                    state.ref_vy = self.ref_state.vy
                    state.ref_vz = self.ref_state.vz
            
                    state.est_vx = self.est_state.vx
                    state.est_vy = self.est_state.vy
                    state.est_vz = self.est_state.vz
                    
                    self.pub_state.publish(state)  
                    
            if timer > self.duration:
                # if maximum flight reached, land the vehicle
                self.pub_land.publish(Empty())
    
        else:
            # if not flying, reference trajectory = desired trajectory
            self.pub.publish(des_state.data[0])
            
        # publish current state from Vicon and also the desired state
        self.pub_d.publish(des_state.data[0])
        
        # update global timer
        self.cur_time = rospy.get_time()
        
    
if __name__ == '__main__':
    print('---Loading Network---')	
    agent = LearningAgent()
    print('---System Ready---')
    rospy.init_node('correction_signal')
    tracking_signal = CorrectionSignal()
    rospy.spin()

