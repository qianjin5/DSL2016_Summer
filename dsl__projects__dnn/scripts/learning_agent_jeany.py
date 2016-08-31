import numpy as np
import tensorflow as tf
import math
import random
import os.path
#import roslib; roslib.load_manifest('dsl__projects__dnn')
#import rospy

#from std_msgs.msg import String
#from data_cleaning import DataCleaner

# For Data Cleaning

class LearningAgent(object):
    def __init__(self):
        tf.reset_default_graph()
        
        self.var_num = 6
        self.varsName = ['x','y','z','xv','yv','zv']
        #
        self.Graph = tf.Graph()
        self.sess = tf.Session(graph = self.Graph)
        
        self.rate = [0.0001 for i in range(self.var_num)]
        
        with self.Graph.as_default():
            self.network = self.createNetwork() 
                  
        self.load_network(self.sess)
        
        #self.pub_Msg = rospy.Publisher('train_msg', String, queue_size=10)
    
    def kill(self):
        self.sess.close()
               
    def weight_variable(self, shape):
        initial = tf.truncated_normal(shape, stddev = 0.1)
        return tf.Variable(initial)

    def bias_variable(self, shape):
        initial = tf.truncated_normal(shape, stddev = 0.1)
        return tf.Variable(initial)
        
    def createNetwork(self):
        # Joe		
        hidden_nodes = [100,100,100,100]
        # Colin
        #hidden_nodes = [128,128]
        input_variables = 23
        output_variables = 1
        
        # input layer
        s = []
        
        # hidden layer
        h = []
        b = []
        h_drop = []
        W = []
        keep_prob = [] 
        y = []
        cost = []
        train_step = []
        readout = []
        alpha = []
        
        L = len(hidden_nodes)
        for j in range(self.var_num):
            s.append(tf.placeholder("float", [None, input_variables]))
            h.append([])
            b.append([])
            h_drop.append([])
            W.append([])
            keep_prob.append(tf.placeholder(tf.float32))
            for i in range(L):
                b[j].append(self.bias_variable([hidden_nodes[i]]))
                if i == 0: 
                    W[j].append(self.weight_variable([input_variables, hidden_nodes[i]]))
                    h[j].append(tf.nn.relu(tf.matmul(s[j], W[j][i]) + b[j][i]))
                else: 
                    W[j].append(self.weight_variable([hidden_nodes[i-1], hidden_nodes[i]]))
                    h[j].append(tf.nn.relu(tf.matmul(h[j][i-1], W[j][i]) + b[j][i]))
                h_drop[j].append(tf.nn.dropout(h[j][i], keep_prob[j]))
            
            # network weights
            b[j].append(self.bias_variable([output_variables]))
            W[j].append(self.weight_variable([hidden_nodes[L-1], output_variables]))

            # readout layer
            readout.append(tf.matmul(h_drop[j][L-1], W[j][L]) + b[j][L])

            alpha.append(0.2)
            
            y.append(tf.placeholder("float", [None, output_variables]))
            cost.append(tf.reduce_mean(tf.square(y[j] - readout[j]) + alpha[j] * tf.square(readout[j])))
            train_step.append(tf.train.AdamOptimizer(self.rate[j]).minimize(cost[j]))
        
        return s, y, cost, train_step, readout, W, b, h, h_drop, keep_prob
    
    def load_network(self, sess):
        with self.Graph.as_default():
            self.sess.run(tf.initialize_all_variables())
            saver = tf.train.Saver()
            checkpoint = tf.train.get_checkpoint_state("saved_networks_quadrotor")
            
            if checkpoint and checkpoint.model_checkpoint_path:
                saver.restore(sess, checkpoint.model_checkpoint_path)
                print "---Successfully loaded: " + str(checkpoint.model_checkpoint_path) + '---'
            else:
                print "---Could Not Find Old Network Weights---"
    
    def save_network(self, sess):
        with self.Graph.as_default():
            saver = tf.train.Saver()
            saver.save(sess, 'saved_networks_quadrotor/Trajectory_Train')

    def save_plot_data(self, filename, time, loss, val_err):
        f = open(filename, "w")
        np.savez(f, time = time, loss = loss, val_err = val_err)
        f.close()
    def load_plot_data(self, filename):
        f = open(filename, "r")
        loaded_file = np.load(f)
        time = loaded_file['time']
        loss = loaded_file['loss']
        val_err = loaded_file['val_err']
        f.close()
        return time, loss, val_err
    
    def Train_all(self, D, val_D, iteration, show):
        for i in range(self.var_num):
            print "\n---------- Training var " + str(i) + " -------------\n"
            self.Train(D, val_D, i, iteration, show)
    
    def Train(self, D, val_D, var_index, iteration, show = False):
        s, y, cost, train_step, readout, W, b, h, h_drop, keep_prob = self.network
        #msg = String()
        if iteration % 1000 == 0:
            self.rate[var_index] = max(self.rate[var_index]/2.0, 0.00001)
        
        #msg.data = '--- Training ' + self.varsName[var_index] + '---'
        #self.pub_Msg.publish(msg)
        
        it_num = 1000
        BATCH = 100
        
        filename = "plot_data" + str(var_index) + ".npz"
        if os.path.isfile(filename):
            plot_dat_time, plot_dat_cost, plot_dat_val = self.load_plot_data(filename)
            plot_dat_time = plot_dat_time.tolist()
            plot_dat_cost = plot_dat_cost.tolist()
            plot_dat_val = plot_dat_val.tolist()
        else:
            plot_dat_time = []
            plot_dat_cost = []
            plot_dat_val = []
        
        for i in range(it_num):
            minibatch = random.sample(D, BATCH)
            data_batch = [[d[0][var_index]] for d in minibatch]
            real_batch = [d[1] for d in minibatch]
            
            #minibatch_val = random.sample(val_D, BATCH_val)
            minibatch_val = val_D
            data_batch_val = [[d[0][var_index]] for d in minibatch_val]
            real_batch_val = [d[1] for d in minibatch_val]
            
            with self.Graph.as_default():
                self.sess.run(train_step[var_index], feed_dict = {
                                                                          y[var_index] : data_batch,
                                                                          s[var_index] : real_batch,
                                                                          keep_prob[var_index]: 0.5})
                                                                          
            with self.Graph.as_default():
                cost_value = self.sess.run(cost[var_index], feed_dict = {
                                                                          y[var_index] : data_batch,
                                                                          s[var_index] : real_batch,
                                                                          keep_prob[var_index]: 1})
            with self.Graph.as_default():
                val_error = self.sess.run(cost[var_index], feed_dict = {y[var_index] : data_batch_val, 
                                                              s[var_index] : real_batch_val,
                                                              keep_prob[var_index] : 1})
                    
            if i % 100 == 0:
                #print "Rate:", self.rate  
                #msg.data = '---Iteration: ' + str(iteration + i) + '---'
                #self.pub_Msg.publish(msg) 
                #msg.data = '---Loss: ' + str(cost_value)
                #self.pub_Msg.publish(msg)    
                print "---Iteration: " + str(iteration + i) + "---"
                print "---Loss: " + str(cost_value) + "---"
                print "Validation error:" + str(val_error) + "---"
            

            if i % 20 == 0:
                plot_dat_time.append(i + iteration)
                plot_dat_cost.append(cost_value)
                plot_dat_val.append(val_error)
            
        self.save_network(self.sess)
        self.save_plot_data(filename, plot_dat_time, plot_dat_cost, plot_dat_val)
        
        if show:
            import matplotlib.pyplot as plt
            plt.plot(plot_dat_time, plot_dat_cost, 'b-', plot_dat_time, plot_dat_val, 'g-', linewidth = 2)
            
            plt.show()
    
    def getRef(self, data, spe = None):
        s, y, cost, train_step, readout, W, b, h, h_drop, keep_prob = self.network
        if spe == None:
            ref = []
            for i in range(6):
                with self.Graph.as_default():
                    ref.append(self.sess.run(readout[i],feed_dict = {s[i] : np.array([data]), keep_prob[i] : 1.0})[0][0])
            return ref
        #else:
            #return readout[spe].eval(feed_dict = {s[spe] : np.array([data]), keep_prob[spe] : 1.0})[0][0]

        
