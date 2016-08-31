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
    
def generate_training_data(file_name, p_c, v_c):
    
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
        x1, y1, z1, x_p1, y_p1, z_p1, R111, R121, R131, R211, R221, R231, R311, R321, R331, p1, q1, r1, z_pp1, xm, ym, zm, x_pm, y_pm, z_pm, R11m, R12m, R13m, R21m, R22m, R23m, R31m, R32m, R33m, pm, qm, rm, z_ppm, xm2, ym2, zm2, x_pm2, y_pm2, z_pm2, R11m2, R12m2, R13m2, R21m2, R22m2, R23m2, R31m2, R32m2, R33m2, pm2, qm2, rm2, z_ppm2, x2, y2, z2, x_p2, y_p2, z_p2, R112, R122, R132, R212, R222, R232, R312, R322, R332, p2, q2, r2, z_pp2 = inp
        roll1, pitch1, yaw1 = R2rpy(np.array([R111, R121, R131, R211, R221, R231, R311, R321, R331]))
        roll2, pitch2, yaw2 = R2rpy(np.array([R112, R122, R132, R212, R222, R232, R312, R322, R332]))
        #inp = np.array([x2 - x1, y2 - y1, z2 - z1, xm - x1, ym - y1, zm - z2, x_pm, y_pm, z_pm, xm2 - x1, ym2 - y1, zm2 - z2, x_pm2, y_pm2, z_pm2, x_p1, y_p1, z_p1, roll1, pitch1, yaw1, p1, q1, r1, z_pp1, x_p2, y_p2, z_p2, roll2, pitch2, yaw2, p2, q2, r2, z_pp2])
        inp = np.array([x2 - x1, y2 - y1, z2 - z1, x_p1, y_p1, z_p1, roll1, pitch1, yaw1, p1, q1, r1, z_pp1, x_p2, y_p2, z_p2, roll2, pitch2, yaw2, p2, q2, r2, z_pp2])
        new_D.append((outp, inp))
        #print outp, inp
    #normalizer(new_D)
    
    #print len(new_D)
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
    
    agent = LearningAgent(False)
    Train = True

    if Train:
        D = generate_training_data(("TrainingDataBoundaryNew" + str(k)), p_c, v_c)
                
        
        new_D = process_data(D)
        total_num = len(new_D)
            
        train_D = []
        validation_D = []
        
        for i in range(total_num):
            if i % 10 != 0:
                train_D.append(new_D[i])
            else:
                validation_D.append(new_D[i])
            
        for j in range(2):
            for i in range(6):
                agent.Train(train_D, validation_D, i, False)
            agent.save_network()
        
        for j in range(4):
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
