### Summer work at Dynamic Systems Lab, University of Toronto Institue for Aerospace Studies in 2016
### Aim to improve quadrotor's trajectory tracking performance using deep neural network

### To install the package
1. Place dsl__projects__dnn in ~/charlottetown/dsl/src/
2. Place saved_networks_quadrotor in ~/.ros/
3. Create folder 'exp_data' under ~/.ros/
4. Build
```
catkin_make install
```
### To run the code
1. To lauch simulator
```
roslaunch dsl__projects__dnn track_reference_signal.launch
```
2. To lauch real flight:
```
roslaunch dsl__projects__dnn track_reference_signal_demo.launch
```
### To train new neural networks:
1. Enbale 'Memorize Experience' during flights to collect data
2. Copy 'Experience.npy' under ~/.ros/ to ~/charlottetown/dsl/src/dsl__projects__dnn/Trainer/
3. Run main.py
4. Copy saved_networks_quadrotor in the same directory to ~/.ros/

#### This is the basic version: only current desired state is used

#### Based on UTIAS DSL Charlottetown deployment: Ubuntu 14.04.04 LTS and ROS Jade 

#### Contents:
- dsl__projects__dnn: to be placed in ~/charlottetown/dsl/src/, main package
- saved_networks_quadrotor: to be placed in ~/.ros/, trained neural networks


