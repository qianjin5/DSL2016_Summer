import numpy as np
from learning_agent import LearningAgent


def save_to_file(file_name, data):
    np.save(file_name, data)

def load_from_file(file_name):
    try:
        return np.load(file_name + ".npy")
    except:
        return None
    
def generate_training_data(file_name, p_c = 1.5, v_c = 1.5):
    try:
	D = load_from_file(file_name)
	return D
    except:
	print 'ERROR NO FILE FOUND!'



def data_transform(d):
    x, y, z, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp, dx, dy, dz, dx_p, dy_p, dz_p = d
    return (np.array([x, y, z, x_p, y_p, z_p, roll, pitch, yaw, p, q, r, z_pp]), np.array([dx, dy, dz, dx_p, dy_p, dz_p]))

def process_data(D):
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

    
if __name__ == '__main__':
    
    D = generate_training_data("Experience_real")
    
    agent = LearningAgent(True)
    
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
        for i in range(6):
	    print 'Loop: ', i
            for j in range(0,6):
                agent.Train(train_D, validation_D, j, False)
            agent.save_network()
