from data_cleaning import DataCleaner
from learning_agent_jeany import LearningAgent
import numpy as np
import random

# Copy Experience.npy from ~/.ros
# After training, copy the networks to ~/.ros

if __name__ == "__main__":
    cleaner = DataCleaner()
    D = cleaner.load_data("Experience.npy")
    print "shape of D =", D.shape
    
    len_D = len(D)
    num_train = len_D - (len_D // 5) # 4/5 for training, 1/5 for validation
    train_D = D[:num_train]
    val_D = D[num_train:]
    cleaned_train_D = cleaner.clean_data(train_D)
    cleaned_val_D = cleaner.clean_data(val_D)
    
    print "length of train_D =", len(train_D)
    print "length of cleaned_train_D =", len(cleaned_train_D)
    agent = LearningAgent()
    n = 2
    for i in range(n-1):
        agent.Train_all(cleaned_train_D, cleaned_val_D, i * 1000, False)
    agent.Train_all(cleaned_train_D, cleaned_val_D, (n-1) * 1000, True)
    
