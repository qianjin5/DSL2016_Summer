### Summer student work at Dynamic Systems Lab, University of Toronto Institue for Aerospace Studies in 2016
### Aim to improve quadrotor's trajectory tracking performance using deep neural network
### Based on UTIAS DSL Charlottetown deployment: Ubuntu 14.04.04 LTS and ROS Jade 

### Contents:
- dsl__projects__dnn: to be placed in ~/charlottetown/dsl/src/, main package
- saved_networks_quadrotor: to be placed in ~/.ros/, trained neural networks
- DSLPath: to be place under /var/www, server files 

### To run code:
1. Visit http://192.168.42.2/mediawiki/index.php/Dynamic_Systems_Lab_DNN_Path_Tracking_Control
2. Read instructions and install all programs
4. Run the code

### To train new neural networks:
1. Enbale 'Memorize Experience' during flights to collect data
2. Copy 'Experience.npy' under ~/.ros/ to ~/charlottetown/dsl/src/dsl__projects__dnn/Trainer/
3. Run main.py
4. Make sure saved_networks_quadrotor folder in the same directory is empty
5. Copy saved_networks_quadrotor folder to ~/.ros/



