from NN_test import NN_test
from NN_generate_data import NN_generate_data
from NN_generate_data2 import NN_generate_data2
from learning_agent import LearningAgent
from generate_randomized_trajectory import generate_randomized_trajectory
import numpy as np
from R2rpy import R2rpy

def save_to_file(file_name, data):
    np.save(file_name, data)

def load_from_file(file_name):
    try:
        return np.load(file_name + ".npy")
    except:
        return None
    
def generate_training_data(file_name, p_c = 1.5, v_c = 1.5):
    
    D = load_from_file(file_name)
    if D == None:
        size = 10
        D = []
        for i in range(size):
            print str(int(i * 1.0 / size * 100) * 1) + "%"
            #data = generator.generate_data(np.array([6, 9, 12]), 0.25, True)
            data = generator.generate_data(p_c, v_c, False)
            D.extend(data)
            print "size:", len(D)
        save_to_file(file_name, np.array(D))
    return D

def plot(x):
    import random
    import numpy
    from matplotlib import pyplot
    bins = numpy.linspace(np.min(x), np.max(x), 150)
    pyplot.hist(x, bins, alpha=0.33, label='NN with feedback')
    pyplot.show()

def normalizer(D):
    for i in range(13):
        x = [d[1][i] for d in D]
        plot(x)


def process_data(D):
    new_D = []    
    
    for d in D:
        outp, inp = d
        #x1, y1, z1, x_p1, y_p1, z_p1, R111, R121, R131, R211, R221, R231, R311, R321, R331, p1, q1, r1, z_pp1, xm, ym, zm, x_pm, y_pm, z_pm, R11m, R12m, R13m, R21m, R22m, R23m, R31m, R32m, R33m, pm, qm, rm, z_ppm, xm2, ym2, zm2, x_pm2, y_pm2, z_pm2, R11m2, R12m2, R13m2, R21m2, R22m2, R23m2, R31m2, R32m2, R33m2, pm2, qm2, rm2, z_ppm2, x2, y2, z2, x_p2, y_p2, z_p2, R112, R122, R132, R212, R222, R232, R312, R322, R332, p2, q2, r2, z_pp2 = inp
        #roll1, pitch1, yaw1 = R2rpy(np.array([R111, R121, R131, R211, R221, R231, R311, R321, R331]))
        #roll2, pitch2, yaw2 = R2rpy(np.array([R112, R122, R132, R212, R222, R232, R312, R322, R332]))
        #inp = np.array([x2 - x1, y2 - y1, z2 - z1, xm - x1, ym - y1, zm - z2, x_pm, y_pm, z_pm, xm2 - x1, ym2 - y1, zm2 - z2, x_pm2, y_pm2, z_pm2, x_p1, y_p1, z_p1, roll1, pitch1, yaw1, p1, q1, r1, z_pp1, x_p2, y_p2, z_p2, roll2, pitch2, yaw2, p2, q2, r2, z_pp2])
        #inp = np.array([x2 - x1, y2 - y1, z2 - z1, x_p1, y_p1, z_p1, roll1, pitch1, yaw1, p1, q1, r1, z_pp1, x_p2, y_p2, z_p2, roll2, pitch2, yaw2, p2, q2, r2, z_pp2])
        #print outp, inp
        new_D.append((outp, inp))
        #print outp, inp
    #normalizer(new_D)
    
    #print len(new_D)
    return new_D

def data_transform(d):
    x, y, z, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp, dx, dy, dz, dx_p, dy_p, dz_p = d
    return (np.array([x, y, z, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp]), np.array([dx, dy, dz, dx_p, dy_p, dz_p]))

def plot_data_xonly(D):
    from matplotlib import pyplot as plt
    t = [i * 0.15 for i in range(len(D))]
    plt.plot(t, [d[0] for d in D], 'r-')
    plt.plot(t, [d[13] for d in D], 'g-')
    plt.plot(t, [d[3] for d in D], 'b-')
    plt.show()


def process_data4(D):
    new_D = []
    hist_len = 3
    length = len(D)
    
    for i in range(length - 1 - 3*(hist_len-1)):
	cur, ref = data_transform(D[i])
	cur2, ref2 = data_transform(D[i + 1])
	
	#curs_f = [data_transform(D[i + 1 + j*4])[0] for j in range(1, hist_len)]
	#refs_f = [data_transform(D[i + 1 + j*4])[1] for j in range(1, hist_len)]
	
	curs_f = [data_transform(D[i + 1 + 1*4])[0], data_transform(D[i + 1 + 2*3])[0]] # 0.6 & 0.9
	refs_f = [data_transform(D[i + 1 + 1*4])[1], data_transform(D[i + 1 + 2*3])[1]]
	
	des_s = np.array([])
	outp = ref - cur2[0:6]
	
	des_s = np.append(des_s, cur[3:])
	
	for j in range(0,hist_len-1):
	    des_s = np.append(des_s, curs_f[j][0:3] - cur[0:3])
	    des_s = np.append(des_s, curs_f[j][3:])
	    
	inp = des_s
	new_D.append((outp, inp))
    return new_D


def process_data3(D):
    new_D = []
    length = len(D)
    pre_len = 5
    for i in range(pre_len, length - pre_len):
	ref_s = np.array([])
	cur, ref = data_transform(D[i])
	cur2, ref2 = data_transform(D[i + 1])
	des_s = np.array([cur])
	#outp = ref - cur2[0:6]
	outp = ref - cur2[0:6]
	for j in range(1, pre_len + 1):
	    cur, ref = data_transform(D[i - j])
	    ref_s = np.append(ref_s, ref)
	for j in range(1, pre_len + 1):    
	    cur, ref = data_transform(D[i + j])
	    des_s = np.append(des_s, cur)
	inp = np.append(ref_s, des_s)
	new_D.append((outp, inp))
    return new_D

def process_data2(D):
    new_D = []
    length = len(D)
    pre_len = 5
    for i in range(length - pre_len):
	ref_s = np.array([])
	outp, inp = D[i]
	f1, f2, f3, x_p1, y_p1, z_p1, roll1, pitch1, yaw1, p1, q1, r1, z_pp1, x_p2, y_p2, z_p2, roll2, pitch2, yaw2, p2, q2, r2, z_pp2 = inp
	new_inp = np.array([f1, f2, f3, x_p1, y_p1, z_p1, roll1, pitch1, yaw1, p1, q1, r1, z_pp1])
	des_s = np.array([new_inp])
	for j in range(1, pre_len + 1):
	    outp, inp = D[i - j]
	    ref_s = np.append(ref_s, outp)
	    outp, inp = D[i + j]
	    f1, f2, f3, x_p1, y_p1, z_p1, roll1, pitch1, yaw1, p1, q1, r1, z_pp1, x_p2, y_p2, z_p2, roll2, pitch2, yaw2, p2, q2, r2, z_pp2 = inp
	    new_inp = np.array([f1, f2, f3, x_p1, y_p1, z_p1, roll1, pitch1, yaw1, p1, q1, r1, z_pp1])
	    des_s = np.append(des_s, new_inp)
	outp, inp = D[i]
	inp = np.append(ref_s, des_s)
	new_D.append((outp, inp))
    return new_D

def plot_stuff(file_name, title):
    data = load_from_file(file_name)
    print data
    fb = [d[0] for d in data]
    wfb = [d[1] for d in data]
    ori = [d[2] for d in data]
    
    max_value = np.max(ori)
    
    import random
    import numpy
    from matplotlib import pyplot

    bins = numpy.linspace(0, 30, 150)

    pyplot.hist(fb, bins, alpha=0.33, label='NN with feedback')
    pyplot.hist(wfb, bins, alpha=0.33, label='NN without feedback')
    pyplot.hist(ori, bins, alpha=0.33, label='original')
    pyplot.legend(loc='upper right')
    pyplot.xlabel('Error')
    pyplot.ylabel('Frequency')
    pyplot.title(title)
    pyplot.show()
    
if __name__ == '__main__':
    generator = NN_generate_data()
    #generator = NN_generate_data2()
    testor = NN_test()
    
    D = generate_training_data("Experience_real")
    #D = generate_training_data("TrainingData")
    
    
    agent = LearningAgent(True)
    #plot_data_xonly(D)
    
    new_D = process_data4(D)
    
    #new_D = np.load("0.6_0.9_cleaned.npy")
    total_num = len(new_D)
    
    #save_to_file('0.6_0.9_processed.npy', new_D)
    
    train_D = []
    validation_D = []
    
    for i in range(total_num):
        if i % 10 != 0:
            train_D.append(new_D[i])
        else:
            validation_D.append(new_D[i])
    
    Train = True

    if Train:
        for i in range(6):
	    print 'Loop: ', i
            for j in range(0,6):
                agent.Train(train_D, validation_D, j, False)
            agent.save_network()
    
    '''for j in range(10):
        amp = j * 0.2 + 0.2
        print "amp =", amp
        error_fb = 0
        error_wfb = 0
        error_ori = 0
        for k in range(10):
            error_fb += testor.Test(agent, True, False, None, True, amp, 0.18, 12, 15, 3)
            error_wfb += testor.Test(agent, False, False, None, False, amp, 0.18, 12, 15, 3)
            error_ori += testor.Test(None, True, False, None, False, amp, 0.18, 12, 15, 3)
        print "[", error_fb/10.0, ",", error_wfb/10.0, ",", error_ori/10.0, "],"
    '''
    '''for k in range(9, 10):   
        fb_error = [0, 0, 0, 0]   
        wfb_error = [0, 0, 0, 0]   
        ori_error = [0, 0, 0, 0]
        for rep in range(5):
            p_c = 0.25 * k
            v_c = 0.25 * k
            D = generate_training_data(("TrainingDataBoundaryNew" + str(k)), p_c, v_c)
            
            agent = LearningAgent(False)
            
            new_D = process_data(D)
            total_num = len(new_D)
                
            train_D = []
            validation_D = []
            
            for i in range(total_num):
                if i % 10 != 0:
                    train_D.append(new_D[i])
                else:
                    validation_D.append(new_D[i])
            
            Train = True
        
            if Train:
                for j in range(2):
                    for i in range(6):
                        agent.Train(train_D, validation_D, i, False)
                    agent.save_network()
        
            for j in range(4):
                amp = j * 2.0 + 1.0
                print "amp =", amp
                fb_error[j] += testor.Test(agent, True, False, None, False, amp, 0.18, 12, 15, 0)
                wfb_error[j] += testor.Test(agent, False, False, None, False, amp, 0.18, 12, 15, 0)
                ori_error[j] += testor.Test(None, True, False, None, False, amp, 0.18, 1, 15, 0)
        for j in range(4):
            fb_error[j] /= 5.0
            wfb_error[j] /= 5.0
            ori_error[j] /= 5.0
            print "amp =", j * 2.0 + 1.0
            print "fb:", fb_error[j]
            print "wfb:", wfb_error[j]
            print "ori:", ori_error[j]
    '''
