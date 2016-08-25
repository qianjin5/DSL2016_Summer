'''
Modified by Qiyang Li
on July/12/2016
'''

param = {'m': 1.447,   # m, mass of vehicle (kg)
         'g': 9.8,     # g, mass normalized gravitational force (m/s^2)
         'L': 0.18,    # L, vehicle arm length (m)
         'K': 0.26,    # K, motor constant, determined experimentally
         'Ix': 0.01152, # Ix, inertia around the body's x-axis (kg-m^2)
         'Iy': 0.01152, # Iy, inertia around the body's y-axis (kg-m^2)
         'Iz': 0.0218,  # Iz, inertia around the body's z-axis (kg-m^2)
         'fmin': 0.17,    # fmin, mass normalized minimum rotor force (m/s^2)
         'fmax': 6.0,     # fmax, mass normalized maximum rotor force (m/s^2)
         'eta': 0.507,   # eta, damping ratio
         'tau_z': 1.0,     # tau_z, time constant for vertical direction
         'tau_Iz': 0.05, #0.20, # tau_Iz, integral time constant for vertical direction
         'tau_yaw': 0.55,    # tau_yaw, time constant for yaw rate
         'tau_Iyaw': 0.010,   # tau_Iyaw, integral time constant for yaw rate
         'eta_y': 0.707,   # eta_y, damping ratio
         'tau_y': 1.7,     # tau_y, time constant for x and y direction
         'tau_Iu': 2.5, #0, # [not used]tau_Iu, integral time constant for x and y dir.
         'tau_p': 0.18,    # tau_p, time constant for roll rate 
         'tau_q': 0.18,    # tau_q, time constant for pitch rate
         'tau_r': 0.1,     # tau_r, time constant for yaw rate 
         'tau_rp': 0.18,    # tau_rp, time constant
         'tau_f': 0,       # [not used]tau_f, time constant for force integration
         'coeff_of_restitution': 0,       # [for bounce]coefficient of restitution
         'outgoing_delay': 100000.0,  # Outgoing delay (in microseconds)
         'incoming_delay': 0,       # Ingoing delay
         'inner_loop_cycle': 8000.0,  # Update rate of inner loop (us)
         'outer_loop_cycle': 15000.0,  # Update rate of outer loop (us)
         'max_euler_angle': 0.32,    # Max_Euler Angle
         'max_yaw_rate': 3.14,    # Max Yaw Rate
         'airdrag_x': 0.35,  # Air Drag Factor in Body X-Direction 0.5
         'airdrag_y': 1.25,  # Air Drag Factor in Body Y-Direction 0.25
         'airdrag_z': 0.3,     # Air Drag Factor in Body Z-Direction
         'inner_oicycle_ratio': 1,       # Inner outer/inner cycle ratio
         'fudge_factor_p': 2.0, #3.125,   # Fudge Factor (Pitch Command) 1.8
         'fudge_factor_r': 1.2, #3.125,   # Fudge Factor (Roll Command) 1.2
         
         'noise_const': 0.001, # noise_const, a const that is porportional to the standard deviation of the noise introduced in the simulator
         };     
