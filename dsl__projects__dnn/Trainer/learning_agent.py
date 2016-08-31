import numpy as np
import tensorflow as tf
import math
import random

class LearningAgent(object):
    def __init__(self, load = True):
        print "Initializing Learning Agent"
        
        tf.reset_default_graph()
        self.sess = tf.InteractiveSession()
        
        self.var_num = 6
        
        self.network = self.createNetwork()
        
        self.sess.run(tf.initialize_all_variables())        
        if load:
            self.load_network()
        print "Initializing Learning Agent (Complete)"
        
    def kill(self):
        self.sess.close()
               
    def weight_variable(self, shape):
        initial = tf.truncated_normal(shape, stddev = 0.1)
        #initial = tf.constant(random.random() - 0.5, shape = shape)
        return tf.Variable(initial)

    def bias_variable(self, shape):
        initial = tf.truncated_normal(shape, stddev = 0.1)
        return tf.Variable(initial)
        
    def createNetwork(self):
        hidden_nodes = [128, 128, 128, 128]
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

            alpha.append(0)
            
            
            y.append(tf.placeholder("float", [None, output_variables]))
            
            cost.append(tf.reduce_mean(tf.square(y[j] - readout[j]) + alpha[j] * tf.square(readout[j])))
            #cost.append(tf.reduce_sum(tf.square(y[j] - readout[j]) + alpha[j] * tf.square(readout[j])))
            #cost.append(tf.reduce_mean(-tf.reduce_sum(y[j] * tf.log(readout[j]))))

            train_step.append(tf.train.AdamOptimizer(0.00003).minimize(cost[j]))
                
        #merged = tf.merge_all_summaries()
        #writer = tf.train.SummaryWriter("/tmp/learning_logs", self.sess.graph)
        return s, y, cost, train_step, readout, W, b, h, h_drop, keep_prob
    
    def load_network(self):
        saver = tf.train.Saver()
        checkpoint = tf.train.get_checkpoint_state("saved_networks_quadrotor")
        if checkpoint and checkpoint.model_checkpoint_path:
            saver.restore(self.sess, checkpoint.model_checkpoint_path)
            print "Successfully loaded:", checkpoint.model_checkpoint_path
        else:
            print "Could not find old network weights"
    
    def save_network(self):
        saver = tf.train.Saver()
        saver.save(self.sess, 'saved_networks_quadrotor/Trajectory_Train')
    
    def Train_all(self, D):
        for i in range(self.var_num):
            self.Train(D, i)
    
    def Train(self, train_D, validation_D, var_index, show = False):
        s, y, cost, train_step, readout, W, b, h, h_drop, keep_prob = self.network
        
        print "var", var_index, "Train!"
        
        it_num = 500
        BATCH = 30
        
        plot_dat_time = []
        plot_dat_cost = []
        
        cost_value = 0
        for i in range(it_num):
            minibatch = random.sample(train_D, BATCH)
            data_batch = [[d[0][var_index]] for d in minibatch]
            real_batch = [d[1] for d in minibatch]
            
            
            #print minibatch[0]
            
            #print data_batch[0], real_batch[0]
            
            train_step[var_index].run(feed_dict = {
            y[var_index] : data_batch,
            s[var_index] : real_batch,
            keep_prob[var_index]: 0.5})
            
            cost_value += cost[var_index].eval(feed_dict = {
            y[var_index] : data_batch,
            s[var_index] : real_batch,
            keep_prob[var_index]: 1})
            
            if i % 250 == 249:
                print "Iteration:", i
                print "Loss:", cost_value/250.0
                cost_value = 0
            
            
            readout_value = readout[var_index].eval(feed_dict = {
            s[var_index] : real_batch,
            keep_prob[var_index]: 1})
            
            #print "Output trajectory:", readout_value[0]
            
            if i % 20 == 0:
                plot_dat_time.append(i)
                plot_dat_cost.append(cost_value)
        
        data_batch = [[d[0][var_index]] for d in validation_D]
        real_batch = [d[1] for d in validation_D]
        real_loss = cost[var_index].eval(feed_dict = {
        y[var_index] : data_batch,
        s[var_index] : real_batch,
        keep_prob[var_index]: 1})
        
        print "validation:", real_loss
        
        if show:
            import matplotlib.pyplot as plt
            plt.plot(plot_dat_time, plot_dat_cost, '-', linewidth=2)
            
            plt.show()
    
    def getRef(self, data, spe = None):
        s, y, cost, train_step, readout, W, b, h, h_drop, keep_prob = self.network
        if spe == None:
            ref = []
            for i in range(6):
                ref.append(readout[i].eval(feed_dict = {s[i] : np.array([data]), keep_prob[i] : 1.0})[0][0])
            return ref
        else:
            return readout[spe].eval(feed_dict = {s[spe] : np.array([data]), keep_prob[spe] : 1.0})[0][0]
        
    '''
    while True:
        print "please input"
        x_A = input()
        y_A = input()
        t_A = input()
        s_A = (x_A, y_A, t_A)
        print agent.Map(s_A)
    '''    
        
