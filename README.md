# Summer work at Dynamic Systems Lab, University of Toronto Institue for Aerospace Studies in 2016
# Aim to improve quadrotor's trajectory tracking performance using deep neural network

# run 'catkin_make install' 
# run 'roslaunch dsl__projects__dnn track_reference_signal.launch' to lauch simulator
# run 'roslaunch dsl__projects__dnn track_reference_signal_demo.launch' for real flights in Vicon lab

# This version employs future states for better modelling of hidden dynamics

# Based on UTIAS DSL Charlottetown deployment: Ubuntu 14.04.04 LTS and ROS Jade 

# Contents:
- dsl__projects__dnn: to be placed in ~/charlottetown/dsl/src/, main package
- saved_networks_quadrotor: to be placed in ~/.ros/, trained neural networks
