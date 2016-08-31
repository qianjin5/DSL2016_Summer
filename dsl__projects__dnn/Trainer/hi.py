import numpy as np

D = np.load('0.6_0.9_cleaned.npy')

count = 0
new_D = []

for i in range(len(D)):
	flag  = True
	for j in range(i+1, len(D)):
		if np.array_equal(D[i],D[j]):
			flag = False
	if flag:
		new_D.append(D[i])
		
		print (1-len(new_D) / float(i)) * 100
	
