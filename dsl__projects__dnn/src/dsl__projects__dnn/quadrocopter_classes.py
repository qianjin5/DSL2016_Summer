#!/usr/bin/env python2

"""
quadrocopter_classes.py

Written By: Adrian Esser and David Wu

Changes:
- Aug 2015 - Vectorized quadrotor state, moved state conversions here

This file contains all classes for the quadrocopter simulation!

This class defines an object for the state of the drone.
The state contains the position, velocity, and acceleration information,
the rotation matrix (which implies pitch, roll, and yaw), and the angular
velocity information.
"""

import numpy as np
import tf.transformations
from dsl__utilities__msg.msg import StateVector


__all__ = ['State', 'Parameters']


class State:

    def __init__(self):

        self.R = np.eye(3)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.acc = np.zeros(3)
        self.omega = np.zeros(3)

    @property
    def quaternion(self):
        """Rotation quaternion corresponding to R."""
        return tf.transformations.quaternion_from_euler(*self.rpy)

    @property
    def rpy(self):
        """Roll, pitch, yaw corresponding to R."""
        return np.array(tf.transformations.euler_from_matrix(self.R))

    @property
    def state_vector(self):
        """Return the state as a StateVector."""
        state = StateVector()
        state.pos[:] = self.pos
        state.vel[:] = self.vel
        state.acc[:] = self.acc
        state.euler[:] = self.rpy
        state.omega_b[:] = self.omega
        state.omega_g[:] = self.R.dot(self.omega)
        return state

    def rpy_to_R(self, rpy):
        return tf.transformations.euler_matrix(*rpy)[:3, :3]


class Parameters:
    """Parameters for quadrotor the define the physics."""

    def __init__(self):

        self.m = 1.477  # m, mass of vehicle (kg)
        self.g = 9.8  # g, mass normalized gravitational force (m/s^2)
        self.L = 0.18  # L, vehicle arm length (m)
        self.K = 0.26  # K, motor constant, determined experimentally
        self.Ix = 0.01152  # Ix, inertia around the body's x-axis (kg-m^2)
        self.Iy = 0.01152  # Iy, inertia around the body's y-axis (kg-m^2)
        self.Iz = 0.0218  # Iz, inertia around the body's z-axis (kg-m^2)
        self.fmin = 0.17  # fmin, mass normalized minimum rotor force (m/s^2)
        self.fmax = 6.0  # fmax, mass normalized maximum rotor force (m/s^2)
        self.vmax = 2.0  # vmax, maximum quadrotor velocity (m/s)
        self.eta = 0.707  # eta, damping ratio
        self.tau_z = 1.0  # tau_z, time constant for vertical direction
        self.tau_Iz = 0.05  # tau_Iz, integral time constant for vertical direction
        self.tau_yaw = 0.55  # tau_yaw, time constant for yaw rate
        self.tau_Iyaw = 0.01  # tau_Iyaw, integral time constant for yaw rate
        self.eta_y = 0.707  # eta_y, damping ratio
        self.tau_y = 1.7  # tau_y, time constant for x and y direction
        self.tau_Iu = 2.5  # tau_Iu, integral time constant for x and y dir.
        self.tau_p = 0.18  # tau_p, time constant for roll rate
        self.tau_q = 0.18  # tau_q, time constant for pitch rate
        self.tau_r = 0.1  # tau_r, time constant for yaw rate
        self.tau_rp = 0.18  # tau_rp, time constant
        self.tau_f = 0.1  # tau_f, time constant for force integration

        #self.CD_bx = 0.55  # Air drag factor in body x direction [dimensionless]
        #self.CD_by = 1.25  # Air drag factor in body y direction [dimensionless]
        #self.CD_bz = 0.3  # Air drag factor in body z direction [dimensionless]

        self.CD_bx = 0.35  # Air drag factor in body x direction [-]
        self.CD_by = 1.25  # Air drag factor in body y direction [-]
        self.CD_bz = 0.3  # Air drag factor in body z direction [-]

        self.incoming_delay = 0.0  # Delay in the signal being sent from quad to computer (us)
        self.outgoing_delay = 100000.0  # Delay in signal being sent from computer to quad (us)
        self.inner_loop_cycle = 8000.0  # Update rate of inner loop (us)
        self.outer_loop_cycle = 15000.0  # Update rate of outer loop (us)

        self.takeoff_height = 0.0  # Takeoff height (m)
        self.takeoff_speed = 0.25  # Takeoff speed (m/s)
        
        self.motor_noise = 0.0000001     # Noise in motor forces (N)
        self.vicon_noise_pos = 0.0000001    # Noise in estimated state of the quadrone
        self.vicon_noise_vel = 0.000001
        #self.vicon_noise_R = 0.01
        self.vicon_noise_omega = 0.000001
