#!/usr/bin/env python2

# Created by Jingxing Qian - Last update: Monday Aug.8, 2016
# Contact me: jingxing.qian@mail.utoronto.ca

# This is a publisher that emits a desired signal which will be further modified by node correction_signal 

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('dsl__projects__dnn')
import rospy
import numpy
from math import *

from std_msgs.msg import Empty, Int64MultiArray, String
from ardrone_autonomy.msg import Navdata
from dsl__utilities__ardrone import DroneStatus
from dsl__projects__dnn import Parameters
from dsl__projects__dnn.msg import StateData
from dsl__projects__dnn.msg import MultiStateData
from dsl__projects__dnn.msg import Equation

from desired_trajectory import DesTrajGetter
#from process_trajectory import process_trajectory


import warnings
warnings.filterwarnings('ignore')

from demo_config import trajinfopath
import json


class ReferenceSignal(object):
    def __init__(self):
        
        # default duration with head and tail
        self.default_duration = 400
	self.traj_start_time = 0.0
        self.traj_end_time = self.default_duration
        # controls reset of variable upon takeoff
        self.can_change = True
        
        # time takeoff
        self.start_time = rospy.get_time()
        self.params = Parameters()
        # outer loop rate
        self.time_step = self.params.outer_loop_cycle / 1e6
        self.state = DroneStatus.Landed
        
        # template for writing local log under ~/.ros/exp_data
        infodict = {"traj_name": 'default_traj_' +str(rospy.get_time()), "traj_id": rospy.get_time(), "avgerr": None, "trial_time": int(self.default_duration)}
        f = open(trajinfopath, "w")
        f.write(json.dumps(infodict))
        f.close()
        
	# default trajectory control
        self.pre_epoch = 0
	
        # trajectory received from server/disk
        self.des_traj = None
        self.dg = DesTrajGetter()  # server connector
        self.user_traj = [1,0,0,0] # use [default, entered, drawn_disk, drawn_server]
        # user draw trajectory counter
        self.traj_index = 0
        # equation trajectory received from GUI
        self.xEq = None
        self.yEq = None
        self.zEq = None

	self.history_length = 3
	self.pub_data = MultiStateData()
	self.pub_data.size = self.history_length
        self.pub_data.data = self.init_data()

        self.sys_ready = True
        
        # Publishers
        self.pub = rospy.Publisher('desired_coordinates_delay', MultiStateData, queue_size=20)
        self.pub_init = rospy.Publisher('init_cond', StateData, queue_size=1)
        self.pub_time = rospy.Publisher('newtime', Int64MultiArray, queue_size=0)
        self.pub_pose = rospy.Timer(rospy.Duration(self.time_step), self.update_state)
        
        # Subscriber
        self.sub_state = rospy.Subscriber('ardrone/navdata', Navdata, self.change_state)
        self.sub_takeoff = rospy.Subscriber('ardrone/takeoff', Empty, self.change_state)
        self.sub_load_traj = rospy.Subscriber('load_traj', Empty, self.load_traj)
        self.sub_load_traj_server = rospy.Subscriber('load_traj_fs', Empty, self.load_traj_fs)
        self.sub_load_traj_server_name = rospy.Subscriber('load_traj_fs_name', String, self.load_traj_fs)
        self.sub_entered_traj = rospy.Subscriber('equation', Equation, self.receive_traj)
        #self.sub_sys_ready = rospy.Subscriber('system_ready', Empty, self.sys_ready)

    
    def sys_ready(self, signal):
        # Not in use currently
        print "---System Ready---"
        self.sys_ready = True
   
    def change_state(self, navdata):
        # reset variables when start to fly or takeoff
        # Note: in original simulator, vehicle has a taking off state which isnt true for real vehicle
        if type(navdata) == Navdata:
            self.state = navdata.state
            if self.can_change == True:
                if self.state == DroneStatus.Flying:
                    self.start_time = rospy.get_time()
                    self.traj_index = 0
                    self.can_change = False
        else:
            self.can_change = True 
            if self.user_traj[0] == 1: 
                time_msg = Int64MultiArray()
                time_msg.data = [self.default_duration, 0, self.default_duration]
                self.pub_time.publish(time_msg)
                
    def receive_traj(self,equation):
        # Use user entered trajectory from GUI
        if not self.sys_ready:
            print '---Try Again Later---'
            return                
        print '---Loading Entered Trajectory---'
        try:
            self.xEq = equation.x
            self.yEq = equation.y
            self.zEq = equation.z
            t = 0
            a = eval(self.xEq)
            b = eval(self.yEq)
            c = eval(self.zEq)
            self.user_traj = [0,1,0,0]
            print '---User Trajectory Loaded Successfully!---'
            print '---You Have Entered:---'
            print '---x(t) = ' + self.xEq + '---'
            print '---y(t) = ' + self.yEq + '---'
            print '---z(t) = ' + self.zEq + '---'
                        
            infodict = {"traj_name": 'equation_traj_' + str(rospy.get_time()), "traj_id": rospy.get_time(), "avgerr": None, "trial_time": int(self.default_duration)}
			
            f = open(trajinfopath, "w")
            f.write(json.dumps(infodict))
            f.close()
            
        except:
            self.user_traj = [1,0,0,0]  
            print '---Trajectory is Invalid. Use Default---'
                
            
    def load_traj(self, signal):  
        # Load saved trajectory from disk under ~/.ros
        if not self.sys_ready:
            print '---Try Again Later---'
            return         
        print '---Loading User Trajectory...---'
        try:
            self.des_traj = numpy.load('Server_traj.npy')
            newtime = int(self.time_step * len(self.des_traj))
            time_msg = Int64MultiArray()
            time_msg.data = [newtime, 0, newtime]
            self.pub_time.publish(time_msg)
            self.user_traj = [0,0,1,0]
            
            infodict = {"traj_name": 'local_traj_' +str(rospy.get_time()), "traj_id": rospy.get_time(), "avgerr": None, "trial_time": newtime}
				
            f = open(trajinfopath, "w")
            f.write(json.dumps(infodict))
            f.close()
            
            print '---User Trajectory Loaded Successfully!---'
            
        except:  
            self.user_traj = [1,0,0,0]  
            print '---No Trajectory File Found! Use Default---'
            
    
    def load_traj_fs(self, signal):
        # Load trajectory from server
        if not self.sys_ready:
            print '---Try Again Later---'
            return    
                 
        try:
	    if signal == Empty():
		print '---Loading Last User Trajectory From Server---'  
		trajobj = self.dg.get_newest_traj()
                traj = trajobj.getDesTraj()
                self.traj_start_time = trajobj.traj_start * self.time_step
                self.traj_end_time = trajobj.traj_end * self.time_step
	    else:
		print '---Loading Trajectory ' + signal.data + ' From Server---'
		trajobj = self.dg.get_traj_by_name(signal.data)
		traj = trajobj.getDesTraj()
                self.traj_start_time = trajobj.traj_start * self.time_step
                self.traj_end_time = trajobj.traj_end * self.time_step
    
	    self.des_traj = traj
	    # save under ~/.ros
	    numpy.save('Server_traj.npy',self.des_traj)
	    
	    newtime = int(self.time_step * len(self.des_traj))
	    # get rid of takeoff and landing from plotting and calculating error
	    time_msg = Int64MultiArray()
	    time_msg.data = [newtime, self.traj_start_time, self.traj_end_time]
	    self.pub_time.publish(time_msg)
	    
	    # Warn: read and write separately. We're adding contents to the file.
	    # "w+" will clear off everything in the file.
	    f = open(trajinfopath, "r")
	    strobj = f.read()
	    f.close()
	    dictobj = json.loads(strobj)
	    dictobj["trial_time"] = newtime
	    f = open(trajinfopath, "w")
	    f.write(json.dumps(dictobj))
	    f.close()
	    
	    self.user_traj = [0,0,0,1]
	    print '---Server Trajectory Loaded Successfully!---'
	    
	except IOError as err:
	    self.user_traj = [1,0,0,0]  
	    print '---No Trajectory Found or Connection Timeout. Use Default---'  
	    print err      
               

    def init_data(self):
        # initialize StateData variables
        pub_data = []
	for i in range(self.history_length):
	    data = StateData()
	    data.x = 0.0
	    data.y = 0.0
	    data.z = 0.0

	    data.pitch = 0.0
	    data.roll = 0.0
	    data.yaw = 0.0

	    data.vx = 0.0
	    data.vy = 0.0
	    data.vz = 0.0

	    data.ax = 0.0
	    data.ay = 0.0
	    data.az = 0.0
	    pub_data.append(data)
        return pub_data

    def update_state(self, event):
        # get next desired state from gen_data and publish it to correction_signal node
        time = rospy.get_time()
        time_stamp = rospy.get_rostime()
        
	for i in range(self.history_length):
	    
	    if i < 2:
		# current and +0.6s
		self.pub_data.data[i].header.stamp = time_stamp + rospy.Duration(i * self.time_step * 10 * 4)
		sim_time = time + i * self.time_step * 10 * 4 - self.start_time
	    else:
		# +0.9s
		self.pub_data.data[i].header.stamp = time_stamp + rospy.Duration(i * self.time_step * 10 * 3)
		sim_time = time + i * self.time_step * 10 * 3 - self.start_time
		
	    self.pub_data.data[i] = self.gen_data(self.pub_data.data[i], sim_time)
        
        self.pub.publish(self.pub_data)
        # if landed, send starting position of traj to calculate offset
        if self.state == DroneStatus.Landed:
            self.init_cond = self.gen_data(self.init_data()[0], 0)
            self.pub_init.publish(self.init_cond)
        
    def gen_data(self, data, time):
        #if self.state == DroneStatus.Flying:		
            #print 'Sim time: ', time		
            
        if self.user_traj[0] == 1:   
             # if not using user-draw trajectory, run default one 
            '''amplitude = 1.0 # Amplitude of oscillation
            phase_shift = 0.0
            frequency = 2.0/15.0 # Frequency
            omega = 2.0*pi*frequency
            z_0 = 1.0
            omega_z = 0.5*omega
            phase_shift_x = 0

            data.x = 0
            data.y = 0
            data.z = 0.5
        
            data.vx = 0
            data.vy = 0
            data.vz = 0
                
            data.ax = 0
            data.ay = 0
            data.az = 0
	    '''
	    
	    epoch = int(time / 15.0) + 1
            
            amplitude = epoch * 0.045 # Amplitude of oscillation
            phase_shift = 0.0
            omega1 = 2.0 * pi / 3.75
            omega2 = 2.0 * pi / 5.0
            omega3 = 2.0 * pi / 7.5
	                
            data.x = amplitude * cos(omega1*time) - amplitude
            data.y = amplitude * cos(omega2*time) - amplitude
            data.z = (amplitude * cos(omega3*time) - amplitude + 4.0)/2.0
    
            data.vx = -omega1 * amplitude * sin(omega1*time)
            data.vy = -omega2 * amplitude * sin(omega2*time)
            data.vz = -omega3 * amplitude * sin(omega3*time)
            
            data.ax = -omega1 ** 2.0 * amplitude * cos(omega1*time)
            data.ay = -omega2 ** 2.0 * amplitude * cos(omega2*time)
            data.az = -omega3 ** 2.0 * amplitude * cos(omega3*time)
	    
	    
            
        elif self.user_traj[1] == 1:
            # if user entered equation trajectory
            t = time  
            # are they valid?
            data.x = eval(self.xEq)
            data.y = eval(self.yEq)
            data.z = eval(self.zEq)
            
            t -= self.time_step
	    # find velocity
            data.vx = (data.x - eval(self.xEq)) / self.time_step
            data.vy = (data.y - eval(self.yEq)) / self.time_step
            data.vz = (data.z - eval(self.zEq)) / self.time_step
            
            data.ax = 0
            data.ay = 0
            data.az = 0
         
        elif self.user_traj[2] + self.user_traj[3] == 1:
            # otherwise, use user drawn trajectory, from disk or server
            
            self.traj_index = int(time / self.time_step)
            
            if self.traj_index > len(self.des_traj) - 1:
                self.traj_index = len(self.des_traj) - 1 
            
	    data.x = self.des_traj[self.traj_index][0]
	    data.y = self.des_traj[self.traj_index][1]
	    data.z = self.des_traj[self.traj_index][2]
                
	    data.vx = self.des_traj[self.traj_index][3]
	    data.vy = self.des_traj[self.traj_index][4]
	    data.vz = self.des_traj[self.traj_index][5]
                
            data.ax = 0
            data.ay = 0
            data.az = 0

             
        
        #if self.state == DroneStatus.Flying:
            #print (data.x,data.y,data.z)
            
        return data

if __name__ == '__main__':
    rospy.init_node('reference_signal')
    tracking_signal = ReferenceSignal()
    rospy.spin()

