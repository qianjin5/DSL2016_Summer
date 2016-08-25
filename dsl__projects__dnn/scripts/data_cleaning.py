import numpy as np
from scipy.stats import norm

class DataCleaner:
    def __init__(self):
	self.alpha = 0.01
	
    def load_data(self, filename):
	return np.load(filename)

    def build_confidence_interval(self, arr):
	# compute mu (loc), sigma (scale)
	mu = np.mean(arr)
	sigma = np.std(arr)
	print "mu =", mu, "sigma =", sigma
	upper_bound = norm.isf(self.alpha / 2, loc = mu, scale = sigma)
	lower_bound = norm.ppf(self.alpha / 2, loc = mu, scale = sigma)
	print "upper_bound =", upper_bound, "lower_bound =", lower_bound
	
	return (lower_bound, upper_bound)
    
    def clean_data(self, D, plot = False):
	cleaned_D = []
	dirty_D = []
	# assume normal distribution
	x_range = self.build_confidence_interval([d[1][0] for d in D])
	y_range = self.build_confidence_interval([d[1][1] for d in D])
	z_range = self.build_confidence_interval([d[1][2] for d in D])

	for d in D:
	    x, y, z = d[1][0], d[1][1], d[1][2]
	    if (x >= x_range[0]) and (x <= x_range[1]) and (y >= y_range[0]) and (y <= y_range[1]) and (z >= z_range[0]) and (z <= z_range[1]):
		cleaned_D.append(d)
	    else:
		    dirty_D.append(d)
	if plot:
	    self.plot_data(D, cleaned_D, dirty_D)
	
	return cleaned_D
	
    def plot_data(self, D, cleaned_D, dirty_D):
	import matplotlib.pyplot as plt
	
	print "len(cleaned_D) =", len(cleaned_D)
	print "len(dirty_D) =", len(dirty_D)
	
	
	delta_x = [d[1][0] for d in cleaned_D]
	delta_y = [d[1][1] for d in cleaned_D]
	delta_z = [d[1][2] for d in cleaned_D]
	
	print "len(delta_x) =", len(delta_x)
	print "len(delta_y) =", len(delta_y)
	print "len(delta_z) =", len(delta_z)

	plt.figure(1)
	plt.subplot(311)
	plt.hist(delta_x, 100)
	plt.subplot(312)
	plt.hist(delta_y, 100)
	plt.subplot(313)
	plt.hist(delta_z, 100)
	plt.show()
    
