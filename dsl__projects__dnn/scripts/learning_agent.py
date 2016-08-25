import numpy as np
import tensorflow as tf
import math
import random
import roslib; roslib.load_manifest('dsl__projects__dnn')
import rospy

from std_msgs.msg import String

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
        
        self.pub_Msg = rospy.Publisher('train_msg', String, queue_size=10)
    
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
        hidden_nodes = [128, 128, 128, 128]
        # Colin
        #hidden_nodes = [128,128,128,128]
        input_variables = 36
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

    
    def Train_all(self, D):
        for i in range(self.var_num):
            self.Train(D, i)
    
    def Train(self, D, var_index, iteration, show = False):
        s, y, cost, train_step, readout, W, b, h, h_drop, keep_prob = self.network
        msg = String()
        if iteration % 1000 == 0:
            self.rate[var_index] = max(self.rate[var_index]/2.0, 0.00001)
        
        msg.data = '---Training ' + self.varsName[var_index] + '---'
        self.pub_Msg.publish(msg)
        
        it_num = 1000
        BATCH = 30
        
        plot_dat_time = []
        plot_dat_cost = []
        
        for i in range(it_num):
            minibatch = random.sample(D, BATCH)
            data_batch = [[d[0][var_index]] for d in minibatch]
            real_batch = [d[1] for d in minibatch]
            
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
            
            if i % 100 == 0:
                #print "Rate:", self.rate  
                msg.data = '---Iteration: ' + str(iteration + i) + '---'
                self.pub_Msg.publish(msg) 
                msg.data = '---Loss: ' + str(cost_value)
                self.pub_Msg.publish(msg)        
            
            with self.Graph.as_default():        
                readout_value = self.sess.run(readout[var_index], feed_dict = {
                                                                            s[var_index] : real_batch,
                                                                            keep_prob[var_index]: 1})
            
            if i % 20 == 0:
                plot_dat_time.append(i)
                plot_dat_cost.append(cost_value)
            
        self.save_network(self.sess)
        
        if show:
            import matplotlib.pyplot as plt
            plt.plot(plot_dat_time, plot_dat_cost, '-', linewidth=2)
            
            plt.show()
      
    def data_transform(self, d):
        x, y, z, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp, dx, dy, dz, dx_p, dy_p, dz_p = d
        return (np.array([x, y, z, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp]), np.array([dx, dy, dz, dx_p, dy_p, dz_p]))
            
    def process_data(self, D):
        new_D = []
        hist_len = 3
        length = len(D)
        
        for i in range(length - 1 - 3*(hist_len-1)):
            cur, ref = self.data_transform(D[i])
            cur2, ref2 = self.data_transform(D[i + 1])
        
            #curs_f = [self.data_transform(D[i + 1 + j*4])[0] for j in range(1, hist_len)]
            #refs_f = [self.data_transform(D[i + 1 + j*4])[1] for j in range(1, hist_len)]
            
            curs_f = [self.data_transform(D[i + 1 + 1*4])[0], self.data_transform(D[i + 1 + 2*3])[0]] # 0.6 & 0.9
            refs_f = [self.data_transform(D[i + 1 + 1*4])[1], self.data_transform(D[i + 1 + 2*3])[1]]
            
            des_s = np.array([])
            outp = ref - cur2[0:6]
            
            des_s = np.append(des_s, cur[3:])
            
            for j in range(0,hist_len-1):
                des_s = np.append(des_s, curs_f[j][0:3] - cur[0:3])
                des_s = np.append(des_s, curs_f[j][3:])
                
            inp = des_s
            new_D.append((outp, inp))
        return new_D
    
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

        
