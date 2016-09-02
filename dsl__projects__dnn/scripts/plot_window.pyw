#!/usr/bin/env python2

# Created by Jingxing Qian - Last update: Monday Aug.25, 2016
# Contact me: jingxing.qian@mail.utoronto.ca

# This is a Qt4-based GUI control panel with toggles and real time plotting 

# Import the ROS and other libraries and files

import roslib; roslib.load_manifest('dsl__projects__dnn')
import rospy

import sys
import math
import random
import numpy as np

from PyQt4 import QtGui, QtCore, QtWebKit

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

from dsl__projects__dnn.msg import State, Equation
from std_msgs.msg import Empty, Int64MultiArray, String

from demo_config import configInfo
import mysql.connector
from mysql.connector import errorcode
import json

import time

class MainWidget(QtGui.QWidget):	
	
	def __init__(self, parent = None):		
		# initialize 
		super(MainWidget, self).__init__() 
		self.initUI()
		self.show()
		
		for i in range(self.num_plot):
			self.plot_frames[i].hide()
			
		self.train_panel.hide()	
		self.has_dnn = False

	def initUI(self):    
		
		# lists for plotting
		self.num_plot = 10
		self.t = []
		self.des = [[],[],[],[],[],[]] # [[x],[y],[z],[vx],[vy],[vz]]
		self.ref = [[],[],[],[],[],[]]
		self.est = [[],[],[],[],[],[]]
        
		self.start_time = rospy.get_time()
		self.timer = QtCore.QTimer(self)
		self.timer.timeout.connect(self.update_fig)
		
		# default fligh time (range of time axis when plotting)
		self.duration = 5
		self.traj_start_time = 0
		self.traj_end_time = self.duration
		self.uploaded = False
		
		# for plotting
		self.frame_plot = [True for i in range(self.num_plot)]
		self.plot_frames = []
		self.fig = []
		self.plots = ['x-t plot','y-t plot','z-t plot','x-y plot','x-z plot','y-z plot','x-y-z plot','vx-t plot','vy-t plot','vz-t plot']
		self.get_data = True 
		
		# Publishers
		#self.pubDNN_Toggle = rospy.Publisher('test', Empty, queue_size=0)
		self.pubDNN_Toggle = rospy.Publisher('DNN_Toggle', Empty, queue_size=0)
		self.pubDNNFeedback_Toggle = rospy.Publisher('dnn_feedback', Empty, queue_size=0)
		self.pubLoadTraj   = rospy.Publisher('load_traj', Empty, queue_size=0)
		self.pubLoadTraj_fs   = rospy.Publisher('load_traj_fs', Empty, queue_size=0)
		self.pubLoadTraj_fs_name   = rospy.Publisher('load_traj_fs_name', String, queue_size=0)
		self.pubLand    = rospy.Publisher('ardrone/land', Empty, queue_size=0)
		self.pubTakeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=0)
		self.pubEquation = rospy.Publisher('equation', Equation, queue_size=0)
		self.pubTime = rospy.Publisher('newtime', Int64MultiArray, queue_size=0)
		self.pubRec = rospy.Publisher('rec_data', Empty, queue_size=0)
		self.pubLearn = rospy.Publisher('trainDNN', Empty, queue_size=0)

		# Subscribers
		self.sub_state = rospy.Subscriber('full_state', State, self.update_data)
		self.sub_takeoff = rospy.Subscriber('ardrone/takeoff', Empty, self.take_off)
		self.sub_land = rospy.Subscriber('ardrone/land', Empty, self.stop_timer)
		self.sub_newtime = rospy.Subscriber('newtime', Int64MultiArray, self.update_time)
		self.sub_train_msg = rospy.Subscriber('train_msg', String, self.update_msg)
		
		# Toggles etc
		self.takeoff = QtGui.QCommandLinkButton('Takeoff (Y)', self)
		self.land = QtGui.QCommandLinkButton('Landed', self)
		self.useDNN = QtGui.QCommandLinkButton('Enable DNN (D)', self)
		self.dnnFeedback = QtGui.QCommandLinkButton('Feedback Mode', self)
		self.loadTraj = QtGui.QCommandLinkButton('Load from Disk (L)', self)
		self.loadTraj_fs = QtGui.QCommandLinkButton('Load from Server (M)', self)
		self.recTraj = QtGui.QCommandLinkButton('Memorize Experience', self)
		self.learnTraj = QtGui.QCommandLinkButton('Train from Experience', self)
		
		self.fs_name = QtGui.QLineEdit()
		self.fs_label = QtGui.QLabel()
		self.fs_label.setText('Traj Name:')
		
		self.takeoff.clicked.connect(self.drone_takeoff)
		self.land.clicked.connect(self.drone_land)
		self.useDNN.clicked.connect(self.use_dnn)
		self.dnnFeedback.clicked.connect(self.use_feedback)
		self.loadTraj.clicked.connect(lambda:self.pubLoadTraj.publish(Empty()))
		self.loadTraj_fs.clicked.connect(self.server_traj)
		self.recTraj.clicked.connect(lambda:self.pubRec.publish(Empty()))
		self.learnTraj.clicked.connect(self.learn_toggle)
		
		self.use_my_path = QtGui.QCheckBox('Use Path Specified Here')
		
		self.flight_layout = QtGui.QHBoxLayout()
		self.time_label = QtGui.QLabel()
		self.time_label.setText('Flight Time')
		self.flight_length = QtGui.QSlider(QtCore.Qt.Horizontal) 
		self.flight_length.setMinimum(5)
		self.flight_length.setMaximum(600)
		self.time_value = QtGui.QLineEdit()
		self.time_value.setText('5s')
		self.time_value.setFixedWidth(40)
		self.flight_layout.addWidget(self.time_label)
		self.flight_layout.addWidget(self.flight_length)
		self.flight_layout.addWidget(self.time_value)
		
		
		self.xeqlabel = QtGui.QLabel()
		self.xeqlabel.setText('x(t):')
		self.yeqlabel = QtGui.QLabel()
		self.yeqlabel.setText('y(t):')
		self.zeqlabel = QtGui.QLabel()
		self.zeqlabel.setText('z(t):')
		
		self.xeq = QtGui.QLineEdit()
		self.yeq = QtGui.QLineEdit()
		self.zeq = QtGui.QLineEdit()

		self.xeqhbox = QtGui.QHBoxLayout()
		self.xeqhbox.addWidget(self.xeqlabel)
		self.xeqhbox.addWidget(self.xeq)
		
		self.yeqhbox = QtGui.QHBoxLayout()
		self.yeqhbox.addWidget(self.yeqlabel)
		self.yeqhbox.addWidget(self.yeq)
		
		self.zeqhbox = QtGui.QHBoxLayout()
		self.zeqhbox.addWidget(self.zeqlabel)
		self.zeqhbox.addWidget(self.zeq)
		
		self.eqbox = QtGui.QVBoxLayout()
		self.eqbox.addLayout(self.xeqhbox)
		self.eqbox.addLayout(self.yeqhbox)
		self.eqbox.addLayout(self.zeqhbox)
		
		self.flight_length.valueChanged.connect(self.slider_change)
		self.time_value.returnPressed.connect(self.time_enter)
		self.use_my_path.stateChanged.connect(lambda:self.use_path_here(self.use_my_path))
		
		self.submit_path = QtGui.QCommandLinkButton('Submit Path', self)
		self.submit_path.clicked.connect(self.submitpath)


		self.box_x_t = QtGui.QCheckBox('x-t plot')
		self.box_y_t = QtGui.QCheckBox('y-t plot')
		self.box_z_t = QtGui.QCheckBox('z-t plot')
		self.box_x_y = QtGui.QCheckBox('x-y plot')
		self.box_x_z = QtGui.QCheckBox('x-z plot')
		self.box_y_z = QtGui.QCheckBox('y-z plot')
		self.box_3d = QtGui.QCheckBox('x-y-z plot')
		self.box_vx_t = QtGui.QCheckBox('vx-t plot')
		self.box_vy_t = QtGui.QCheckBox('vy-t plot')
		self.box_vz_t = QtGui.QCheckBox('vz-t plot')
		
		self.boxes = [self.box_x_t, self.box_y_t, self.box_z_t, self.box_x_y, self.box_x_z, self.box_y_z, self.box_3d, self.box_vx_t, self.box_vy_t, self.box_vz_t]
        
		self.select_all = QtGui.QCommandLinkButton('Select All', self)
		self.deselect_all = QtGui.QCommandLinkButton('Deselect All', self)
		self.clear_all = QtGui.QCommandLinkButton('Clear All', self)
        
		self.start_all = QtGui.QCommandLinkButton('Start Plotting All', self)
		self.stop_all = QtGui.QCommandLinkButton('Stop Plotting All', self)
		self.exbt = QtGui.QCommandLinkButton('Exit ROS', self)
        
        
		self.layoutTT = QtGui.QHBoxLayout()
        
		self.layoutTTL = QtGui.QVBoxLayout()
		self.layoutTTR = QtGui.QVBoxLayout()
        
		self.layoutLT = QtGui.QVBoxLayout()
		self.layoutRT = QtGui.QVBoxLayout()
		self.layoutT = QtGui.QHBoxLayout()
		
		self.layoutLB = QtGui.QVBoxLayout()
		self.layoutRB = QtGui.QVBoxLayout()
		self.layoutB = QtGui.QHBoxLayout()
		
		self.layout = QtGui.QVBoxLayout()
		
		self.layoutLT.addWidget(self.box_x_t)
		self.layoutLT.addWidget(self.box_y_t)
		self.layoutLT.addWidget(self.box_z_t)
		self.layoutLT.addWidget(self.box_3d)
		self.layoutLT.addWidget(self.box_vy_t)
		
		self.layoutRT.addWidget(self.box_x_y)
		self.layoutRT.addWidget(self.box_x_z)
		self.layoutRT.addWidget(self.box_y_z)
		self.layoutRT.addWidget(self.box_vx_t)
		self.layoutRT.addWidget(self.box_vz_t)
		
		self.layoutT.addLayout(self.layoutLT)
		self.layoutT.addLayout(self.layoutRT)
		
		self.layoutLB.addWidget(self.select_all)
		self.layoutLB.addWidget(self.deselect_all)
		self.layoutLB.addWidget(self.clear_all)
		self.layoutRB.addWidget(self.start_all)
		self.layoutRB.addWidget(self.stop_all)
		self.layoutRB.addWidget(self.exbt)
		
		self.layoutB.addLayout(self.layoutLB)
		self.layoutB.addLayout(self.layoutRB)
		
		
		self.layoutTTL.addWidget(self.takeoff)
		self.layoutTTL.addWidget(self.useDNN)
		self.layoutTTL.addWidget(self.recTraj)
		self.layoutTTL.addWidget(self.learnTraj)
		self.layoutTTR.addWidget(self.land)
		self.layoutTTR.addWidget(self.dnnFeedback)
		self.layoutTTR.addWidget(self.loadTraj)
		self.layoutTTR.addWidget(self.loadTraj_fs)
		
		self.layoutTTR_fs = QtGui.QHBoxLayout()
		self.layoutTTR_fs.addWidget(self.fs_label)
		self.layoutTTR_fs.addWidget(self.fs_name)
		self.layoutTTR.addLayout(self.layoutTTR_fs)
		
		self.layoutTT.addLayout(self.layoutTTL)
		self.layoutTT.addLayout(self.layoutTTR)
		
		self.layout.addLayout(self.layoutTT)
		self.layout.addLayout(self.flight_layout)
		self.layout.addLayout(self.eqbox)
		self.layout.addWidget(self.submit_path)
		self.layout.addLayout(self.layoutT)
		self.layout.addLayout(self.layoutB)
		
		self.box_x_t.stateChanged.connect(lambda:self.box_change(self.box_x_t))
		self.box_y_t.stateChanged.connect(lambda:self.box_change(self.box_y_t))
		self.box_z_t.stateChanged.connect(lambda:self.box_change(self.box_z_t))
		self.box_x_y.stateChanged.connect(lambda:self.box_change(self.box_x_y))
		self.box_x_z.stateChanged.connect(lambda:self.box_change(self.box_x_z))
		self.box_y_z.stateChanged.connect(lambda:self.box_change(self.box_y_z))
		self.box_3d.stateChanged.connect(lambda:self.box_change(self.box_3d))
		self.box_vx_t.stateChanged.connect(lambda:self.box_change(self.box_vx_t))
		self.box_vy_t.stateChanged.connect(lambda:self.box_change(self.box_vy_t))
		self.box_vz_t.stateChanged.connect(lambda:self.box_change(self.box_vz_t))
		
		self.start_all.clicked.connect(self.all_start)
		self.stop_all.clicked.connect(self.all_stop)
		self.select_all.clicked.connect(self.selectAll)
		self.deselect_all.clicked.connect(self.deselectAll)
		self.clear_all.clicked.connect(self.clear)
		self.exbt.clicked.connect(self.close)
		#self.exbt.clicked.connect(QtCore.QCoreApplication.instance().quit)

		self.setGeometry(800, 1000, 400, 650)
		self.center()
		self.setLayout(self.layout)
		self.setWindowTitle('Welcome!')
	
		for i in range(self.num_plot):
			# create plotting windows
			frame = self.new_plot_frame(i)
			self.plot_frames.append(frame)
			
		self.train_panel = 	self.new_train_panel()

	def new_plot_frame(self, index):
		frame = QtGui.QFrame()
		frame.setFrameStyle(QtGui.QFrame.StyledPanel|QtGui.QFrame.Sunken)
		frame.setGeometry(500, 400, 900, 600)
		frame.setWindowTitle(self.plots[index])
		
		frame.start = QtGui.QPushButton('Start '+ self.plots[index], self)
		frame.stop = QtGui.QPushButton('Stop ' + self.plots[index], self)
		frame.scroll = QtGui.QCheckBox('Auto Scroll') # not in use
		frame.scroll.setChecked(True)
		
		frame.figure = Figure()
		frame.canvas = FigureCanvas(frame.figure)
		
		if index == 6:
			frame.plot = frame.figure.add_subplot(111, projection='3d')     	
		else:
			frame.plot = frame.figure.add_subplot(111)
			                           
		self.fig.append(frame.plot)
		frame.toolbar = NavigationToolbar(frame.canvas, frame)

		frame.hlayout = QtGui.QHBoxLayout()
		frame.hlayout.addWidget(frame.start)
		frame.hlayout.addWidget(frame.stop)
		frame.hlayout.addWidget(frame.scroll)
		
		frame.vlayout = QtGui.QVBoxLayout()
		frame.vlayout.addLayout(frame.hlayout)
		frame.vlayout.addWidget(frame.toolbar)
		frame.vlayout.addWidget(frame.canvas)
		
		frame.start.clicked.connect(lambda:self.start(index))
		frame.stop.clicked.connect(lambda:self.stop(index))
		frame.scroll.stateChanged.connect(self.scroll)	
		
		frame.setLayout(frame.vlayout)
		
		return frame
	
	def new_train_panel(self):
		# for training DNN from experience, use may cause freeze due to resource issue
		panel = QtGui.QFrame()
		panel.setFrameStyle(QtGui.QFrame.StyledPanel|QtGui.QFrame.Sunken)
		
		panel.setGeometry(450, 250, 500, 300)
		panel.setWindowTitle('DNN Trainer')
		
		panel.text = QtGui.QTextBrowser(self)
		panel.exitbt = QtGui.QPushButton('Exit', self)
		
		panel.hlayout1 = QtGui.QHBoxLayout()
		panel.hlayout1.addWidget(panel.exitbt)
		
		
		panel.vlayout = QtGui.QVBoxLayout()
		panel.vlayout.addWidget(panel.text)
		panel.vlayout.addLayout(panel.hlayout1)	
		
		panel.setLayout(panel.vlayout)
		
		panel.exitbt.clicked.connect(lambda:panel.hide())
		
		return panel
		
	def closeEvent(self, event):        		
		reply = QtGui.QMessageBox.question(self, 'Message', "Do you want to quit ROS?", QtGui.QMessageBox.No | QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)

		if reply == QtGui.QMessageBox.Yes:
			event.accept()
		else:
			event.ignore()
            
	def stop_timer(self,msg):
		# when landing the vehicle
		self.timer.stop()
		self.takeoff.setText('Takeoff (Y)')
		self.land.setText('Landed')
		# push to server once
		time.sleep(0.2)
		if not self.uploaded:
			self.pushToDB(self.est[0:3], self.ref[0:3], self.des[0:3])    
			
	def center(self):        		
		qr = self.frameGeometry()
		cp = QtGui.QDesktopWidget().availableGeometry().center()
		qr.moveCenter(cp)
		self.move(qr.topLeft()) 
		
	def drone_takeoff(self):
		self.pubTakeoff.publish(Empty())
		
	def drone_land(self):
		self.takeoff.setText('Takeoff (Y)')
		self.land.setText('Landed')
		self.pubLand.publish(Empty())
		
	def use_dnn(self):
		self.pubDNN_Toggle.publish(Empty())
		if self.useDNN.text() == 'Enable DNN (D)':
			self.useDNN.setText('Disable DNN (D)')
			self.has_dnn = True
		elif self.useDNN.text() == 'Disable DNN (D)':
			self.useDNN.setText('Enable DNN (D)')
			self.has_dnn = False
			
	def use_feedback(self):
		self.pubDNNFeedback_Toggle.publish(Empty())
		if self.dnnFeedback.text() == 'Feedback Mode':
			self.dnnFeedback.setText('Non-feedback Mode')
		elif self.dnnFeedback.text() == 'Non-feedback Mode':
			self.dnnFeedback.setText('Feedback Mode')		
		
	def selectAll(self):
		for i in range(self.num_plot):
			self.boxes[i].setChecked(True)
		
	def deselectAll(self):
		for i in range(self.num_plot):
			self.boxes[i].setChecked(False)	
		
	def box_change(self, opt):
		# show plotting window when selected, hide if deselected
		index = self.plots.index(opt.text())
		if opt.isChecked() == True:
			self.plot_frames[index].show()
			self.plot(index,True)
		else:
			self.plot_frames[index].hide()
			
	def learn_toggle(self):
		# Can train if landed
		if self.takeoff.text() == 'Takeoff (Y)':
			self.pubLearn.publish(Empty())
			self.train_panel.show()
		
	def all_start(self):
		print '---All Plots Started---'
		self.get_data = True
		
	def all_stop(self):
		print '---All Plots Stopped---'
		self.get_data = False
		
	def start(self,i):
		print '---' + self.plots[i] + ' plot started---'
		self.frame_plot[i] = True
		
	def stop(self,i):
		print '---' + self.plots[i] + ' plot stopped---'
		self.frame_plot[i] = False
		
	def scroll(self):
		pass
		
	def use_path_here(self,path):
		if path.isChecked() == True:
			pass
		else:
			pass
		
	def slider_change(self):
		self.time_value.setText(str(self.flight_length.value()) + 's')	
		
	def time_enter(self):
		try:
			self.flight_length.setValue(min(max(int(self.time_value.text()),5),600))
		except:
			self.time_value.setText('Err')
			
	def submitpath(self):
		# submit equation path
		time = Int64MultiArray()
		self.duration = int(self.flight_length.value())
		self.traj_start_time = 0
		self.traj_end_time = self.duration
		time.data = [self.duration, self.traj_start_time, self.traj_end_time]
		
		equation = Equation()
		equation.x = str(self.xeq.text())
		equation.y = str(self.yeq.text())
		equation.z = str(self.zeq.text())
		if self.time_value.text() != 'Err':
			self.pubTime.publish(time)
			self.pubEquation.publish(equation)
			
	def server_traj(self):
		if self.fs_name.text() == '':
			self.pubLoadTraj_fs.publish(Empty())
		else:
			msg = String()
			msg.data = str(self.fs_name.text())
			self.pubLoadTraj_fs_name.publish(msg)

	def update_time(self, msg):
		self.duration = int(msg.data[0])
		self.time_value.setText(str(self.duration)+'s')
		self.flight_length.setValue(self.duration)
		self.traj_start_time = msg.data[1]
		self.traj_end_time = msg.data[2]
		
		
	def update_msg(self,msg):
		# show text for training panel
		self.train_panel.text.append(msg.data)
		
	def	take_off(self,msg):
		self.takeoff.setText('Flying')
		self.land.setText('Land (H)')
		self.start_time = rospy.get_time()
		self.uploaded = False
		self.clear()
		
	def clear(self,msg=None):	
		# force clean all plots at takeoff or toggle clicked		
		self.t = []
		self.des = [[],[],[],[],[],[]]
		self.ref = [[],[],[],[],[],[]]
		self.est = [[],[],[],[],[],[]]
		self.timer.stop()
		self.frame_plot = [True for i in range(self.num_plot)]
		if self.get_data == True:
			print '---All plots started---'
		else:
			print '---Press "Start Plotting All" to start plotting---'
		for i in range(self.num_plot):
			if not self.plot_frames[i].isVisible():
				self.boxes[i].setChecked(False)	
			self.fig[i].cla()
			self.plot(i,True)
		
	def update_data(self,state):
		# update data from correction signal
		cur_time = rospy.get_time() - self.start_time
		#if self.traj_start_time < cur_time < self.traj_end_time:
		if self.get_data:
			self.t.append(state.t)
			self.des[0].append(state.des_x)
			self.des[1].append(state.des_y)
			self.des[2].append(state.des_z)
			self.des[3].append(state.des_vx)
			self.des[4].append(state.des_vy)
			self.des[5].append(state.des_vz)
			
			self.ref[0].append(state.ref_x)
			self.ref[1].append(state.ref_y)
			self.ref[2].append(state.ref_z)
			self.ref[3].append(state.ref_vx)
			self.ref[4].append(state.ref_vy)
			self.ref[5].append(state.ref_vz)
			
			self.est[0].append(state.est_x)
			self.est[1].append(state.est_y)
			self.est[2].append(state.est_z)
			self.est[3].append(state.est_vx)
			self.est[4].append(state.est_vy)
			self.est[5].append(state.est_vz)
        
			if len(self.t) == 1:
				self.timer.start(100)
		
	def update_fig(self):
		# update window if the specific frame is shown  
		if self.get_data:          
			for i in range(self.num_plot):
				if self.plot_frames[i].isVisible():
					self.plot(i)
		
		
	def plot(self,i,forced_clean = False):
        
		if self.frame_plot[i] or forced_clean:
        
			f = self.fig[i]
			l = 1 if self.t else 0           
                
        
			if i == 0:
			#x-t
			
				f.plot(np.array(self.t), np.array(self.est[0]), 'b',label = 'Actual X-Position' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.ref[0]), 'g--',label = 'Reference X-Position' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.des[0]), 'r--',label = 'Desired X-Position' if l == 0 else '')
                
				f.set_xlabel('Time (s)')
				f.set_ylabel('Position (m)')
				f.axis([0, int(self.duration), -3, 3])    
			
			elif i == 1:
			#y-t
			
				f.plot(np.array(self.t), np.array(self.est[1]), 'b',label = 'Actual Y-Position' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.ref[1]), 'g--',label = 'Reference Y-Position' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.des[1]), 'r--',label = 'Desired Y-Position' if l == 0 else '')
                
				f.set_xlabel('Time (s)')
				f.set_ylabel('Position (m)')
				f.axis([0, int(self.duration), -3, 3]) 
	
			elif i == 2:
			#z-t
			
				f.plot(np.array(self.t), np.array(self.est[2]), 'b',label = 'Actual Z-Position' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.ref[2]), 'g--',label = 'Reference Z-Position' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.des[2]), 'r--',label = 'Desired Z-Position' if l == 0 else '')
                
				f.set_xlabel('Time (s)')
				f.set_ylabel('Position (m)')
				f.axis([0, int(self.duration), 0, 2.5]) 
			
			elif i == 3:
			#x-y
			
				f.plot(np.array(self.est[0]), np.array(self.est[1]), 'b',label = 'Actual X-Y Position' if l == 0 else '')
				f.plot(np.array(self.ref[0]), np.array(self.ref[1]), 'g--',label = 'Reference X-Y Position' if l == 0 else '')
				f.plot(np.array(self.des[0]), np.array(self.des[1]), 'r--',label = 'Desired X-Y Position' if l == 0 else '')
                
				f.set_xlabel('X Position (m)')
				f.set_ylabel('Y Position (m)')
				f.axis([-3, 3, -3, 3]) 
			
			elif i == 4:
			#x-z
			
				f.plot(np.array(self.est[0]), np.array(self.est[2]), 'b',label = 'Actual X-Z Position' if l == 0 else '')
				f.plot(np.array(self.ref[0]), np.array(self.ref[2]), 'g--',label = 'Reference X-Z Position' if l == 0 else '')
				f.plot(np.array(self.des[0]), np.array(self.des[2]), 'r--',label = 'Desired X-Z Position' if l == 0 else '')
                
				f.set_xlabel('X Position (m)')
				f.set_ylabel('Z Position (m)')
				f.axis([-3, 3, 0, 2.5])
			
			elif i == 5:
			#y-z
			
				f.plot(np.array(self.est[1]), np.array(self.est[2]), 'b',label = 'Actual Y-Z Position' if l == 0 else '')
				f.plot(np.array(self.ref[1]), np.array(self.ref[2]), 'g--',label = 'Reference Y-Z Position' if l == 0 else '')
				f.plot(np.array(self.des[1]), np.array(self.des[2]), 'r--',label = 'Desired Y-Z Position' if l == 0 else '')
                
				f.set_xlabel('Y Position (m)')
				f.set_ylabel('Z Position (m)')
				f.axis([-3, 3, 0, 2.5])
			
			elif i == 6:
			#x-y-z
			
				f.plot(np.array(self.est[0]),np.array(self.est[1]),np.array(self.est[2]), 'b',label = 'Actual Trajectory' if l == 0 else '')
				f.plot(np.array(self.ref[0]),np.array(self.ref[1]),np.array(self.ref[2]), 'g--',label = 'Reference Trajectory' if l == 0 else '')
				f.plot(np.array(self.des[0]),np.array(self.des[1]),np.array(self.des[2]), 'r--',label = 'Desired Trajectory' if l == 0 else '')

				f.set_xlabel('X Position (m)')
				f.set_ylabel('Y Position (m)')
				f.set_zlabel('Z Position (m)')
				f.set_xlim3d(-3, 3)
				f.set_ylim3d(-3, 3)
				f.set_zlim3d(0, 2.5)
				
			if i == 7:
			#vx-t
			
				f.plot(np.array(self.t), np.array(self.est[3]), 'b',label = 'Actual X-Velocity' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.ref[3]), 'g--',label = 'Reference X-Velocity' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.des[3]), 'r--',label = 'Desired X-Velocity' if l == 0 else '')
                
				f.set_xlabel('Time (s)')
				f.set_ylabel('Velocity (m/s)')
				f.axis([0, int(self.duration), -2, 2])    
			
			elif i == 8:
			#vy-t
			
				f.plot(np.array(self.t), np.array(self.est[4]), 'b',label = 'Actual Y-Velocity' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.ref[4]), 'g--',label = 'Reference Y-Velocity' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.des[4]), 'r--',label = 'Desired Y-Velocity' if l == 0 else '')
                
				f.set_xlabel('Time (s)')
				f.set_ylabel('Velocity (m/s)')
				f.axis([0, int(self.duration), -2, 2]) 
	
			elif i == 9:
			#vz-t
			
				f.plot(np.array(self.t), np.array(self.est[5]), 'b',label = 'Actual Z-Velocity' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.ref[5]), 'g--',label = 'Reference Z-Velocity' if l == 0 else '')
				f.plot(np.array(self.t), np.array(self.des[5]), 'r--',label = 'Desired Z-Velocity' if l == 0 else '')
                
				f.set_xlabel('Time (s)')
				f.set_ylabel('Velocity (m/s)')
				f.axis([0, int(self.duration), -2, 2]) 
		
			f.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=3, mode="expand", borderaxespad=0.) 
			f.grid(True, color='grey')       
		     				
			self.plot_frames[i].canvas.draw()
			#self.plot_frames[i].figure.savefig('saved_img/img_'+str(len(self.t))+'.png')
			

	def pushToDB(self, est, ref, des):

			traj_id = -1
			trajname = "Traj name"
			avgerr = 10000
			
			try:
				from demo_config import trajinfopath
				from demo_config import phys_range
				f = open(trajinfopath, "r")
				trajinfo = f.read()
				f.close()
				dictobj = json.loads(trajinfo)
				#print dictobj['traj_id'], dictobj['traj_name'], dictobj['avgerr']
				traj_id = int(dictobj['traj_id'])
				trajname = str(dictobj['traj_name'])
				avgerr = float(dictobj['avgerr']) # Assume the value of 'avgerr' is not None
				dictobj["phys_range"] = phys_range
				dictobj["est_traj"] = est
				dictobj["ref_obj"] = ref
				dictobj["des_obj"] = des
				dictobj["has_dnn"] = self.has_dnn
				from demo_config import respathbase
				fname = respathbase + "%d.json" %traj_id
				f = open(fname, "w+")
				f.write(json.dumps(dictobj))
				f.close()
				print "---Log Successfully Stored to File---"

			except IOError:
				rospy.logerr("---Log Record Failed---")


			config = configInfo
			
			try:
				print '---Updating Server. Please Wait...---'
				cnx = mysql.connector.connect(**config)
				cursor = cnx.cursor()

				if self.has_dnn:

					query = ('''INSERT into DSLPath_res_dnn (traj_id, trajname, est_traj, ref_traj, des_traj, avgerr, phys_range)
						VALUES (%d, %s, %s, %s, %s, %f, %s);'''
						%(traj_id, self.mystr(trajname), self.mystr(est), self.mystr(ref),
						self.mystr(des), avgerr,
						self.mystr(phys_range)))
				else:
					query = ('''INSERT into DSLPath_res_nd (traj_id, trajname, est_traj, ref_traj, des_traj, avgerr, phys_range)
						VALUES (%d, %s, %s, %s, %s, %f, %s);'''
						%(traj_id, self.mystr(trajname), self.mystr(est), self.mystr(ref),
						self.mystr(des), avgerr,
						self.mystr(phys_range)))
				try:
					cursor.execute(query)
				except mysql.connector.IntegrityError as err:
					print "---Result for This Trajectory Has Been Submitted Before---"
				cnx.commit()
				cursor.close()
				cnx.close()
				print "---Experience Uploaded to Server---"
				
			except IOError as err:
				#print err
				print "---Timeout When Connecting to Server---"

			self.uploaded = True

		
	
	def mystr(self, obj):
		'''Inputs an obj, outputs a string with quotes wrapping it
		'''
		return '"' + str(obj) + '"'

			
def main():    
	app = QtGui.QApplication(sys.argv)
	ex = MainWidget()
	sys.exit(app.exec_())

if __name__ == '__main__':
	
    rospy.init_node('plot_window')
    main()
    rospy.spin()
	

