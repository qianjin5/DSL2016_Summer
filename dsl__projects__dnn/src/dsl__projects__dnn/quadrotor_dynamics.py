"""
TEST
"""

from __future__ import division, print_function
import numpy as np
import quadrocopter_classes

__all__ = ['QuadrotorDynamics', 'wind_creator', 'random_disturbance_creator']

'''
Modification:
Qiyang Li
July 19th

Added Motor Force Clamping (now params fmax and fmin are in use)
Added Corresponding matlab function for future comparison

'''


class QuadrotorDynamics(object):
    """Implement the quadrotor dynamics and states (independent of gazebo).

    Parameters:
    -----------
    pos: 3d array
        Initial position of quadrotor
    vel: 3d array
        Initial velocity of quadrotor
    acc: 3d array
        Initial acceleration of quadrotor
    R: 3x3 array
        Initial rotation matrix
    external_forces: list
        a list of callables that take the state as input and return forces on
        the quadrotor in global coordinates.
    """

    def __init__(self, pos=None, vel=None, acc=None, R=None,
                 external_forces=None):
        self.state = quadrocopter_classes.State()
        self.params = quadrocopter_classes.Parameters()

        if external_forces is None:
            self.external_forces = ()
        else:
            self.external_forces = external_forces

        if pos is not None:
            self.state.pos = pos.copy()
        if vel is not None:
            self.state.vel = vel.copy()
        if acc is not None:
            self.state.acc = acc.copy()
        if R is not None:
            self.state.R = R.copy()

    ############# MATLAB SIMLUATOR FUNCTIONS #################
    def clamp(self, value, lb, ub):
        # Takes a value, a lower bound (lb) and an upper bound (ub) and returns the
        # value if it is between the lb and ub, the lb if the value is below the
        # lb, and the ub if the value is above the upper bound.
        result = max(min(value, ub), lb)
        return result
        
    def convertstate_forward(self, state):
        return np.append(np.append(state.pos, state.vel), state.R)
        
    def convertcmd(self, outer_cmd, cur_state):
        import math
        import numpy as np
        # This function converts the roll, pitch, yaw_rate and z_dot commanded
        # through the cmd_vel topic to the p, q, r and thrust commands used by the
        # inner controller.
        
        # r commanded = yaw_rate commanded
        r_cmd = outer_cmd[2]
        
        # Desired z acceleration is (1/tau_z)*(z_dot_des - z_dot)
        z_ddot_des = (1.0 / self.params.tau_Iz) * (outer_cmd[3] - cur_state[5])

        # Commanded thrust is (g + z_ddot_des)/R33
        thrust_cmd = (self.params.g + z_ddot_des) / cur_state[14]
        
        # Calculate the desired R13 and R23
        pitch_cur = math.asin(self.clamp(-cur_state[12], -1.0, 1.0))
        yaw_cur = math.acos(self.clamp(cur_state[6] / math.cos(pitch_cur), -1.0, 1.0))
        yaw_des = outer_cmd[2] * self.params.tau_Iyaw + yaw_cur
        R13_des = math.sin(yaw_des) * math.sin(outer_cmd[0]) + math.cos(yaw_des) * math.cos(outer_cmd[0]) * math.sin(outer_cmd[1])
        R23_des = math.cos(outer_cmd[0]) * math.sin(yaw_des) * math.sin(outer_cmd[1]) - math.cos(yaw_des) * math.sin(outer_cmd[0])

        # p_cmd = (R21*(R13_des-R13) - R11*(R23_des-R23))/(R33*tau_rp)
        p_cmd = (cur_state[9] * (R13_des-cur_state[8]) - cur_state[6] * (R23_des-cur_state[11])) / (cur_state[14] * self.params.tau_rp)

        # q_cmd = (R22*(R13_des-R13) - R12*(R23_des-R23))/(R33*tau_rp)
        q_cmd = (cur_state[10] * (R13_des-cur_state[8]) - cur_state[7] * (R23_des-cur_state[11])) / (cur_state[14] * self.params.tau_rp)

        # Construct inner command vector
        return p_cmd,  q_cmd, r_cmd, thrust_cmd
    def determineforces(self, cur_state, inner_cmd):
        import numpy as np
        # This function determines the equivalent commanded forces given commands
        # for thrust, p, q and r.


        # Determine A matrix: L = param(3), K = param(4), m = param(1), tau_p =
        # param(18), tau_q = param(19), tau_r = param(20), Ix = param(5), Iy =
        # param(6), Iz = param(7)
        A = [[0, self.params.L, 0, -self.params.L], 
         [-self.params.L, 0, self.params.L, 0], 
         [self.params.K, -self.params.K, self.params.K, -self.params.K], 
         [(1.0/self.params.m), (1.0/self.params.m), (1.0/self.params.m), (1.0/self.params.m)]]
         
        # Determine b matrix
        J = np.diag([self.params.Ix, self.params.Iy, self.params.Iz], 0)
        #J = np.diag([param['Ix'], param['Iy'], param['Iz']])
        
        # inertial matrix
        omega = [cur_state[15], cur_state[16], cur_state[17]] # angular velocity
        rate_vector = [(1.0 / self.params.tau_p) * (inner_cmd[0] - cur_state[15]),
               (1.0 / self.params.tau_q) * (inner_cmd[1] - cur_state[16]), 
               (1.0 / self.params.tau_r) * (inner_cmd[2] - cur_state[17])]
        
        b = np.append(np.add(np.dot(J, rate_vector), np.cross(omega , np.dot(J, omega))),inner_cmd[3])
        #b = J.dot(rate_vector) + np.cross(omega, J.dot(omega))
        #b = np.concatenate(b, inner_cmd[3])
        
        # Solve for the forces
        #forces = np.dot(np.linalg.inv(A), b)
        forces = np.linalg.solve(A, b)
        
        # Clamp the force between maxmium and minimum values
        forces[0] = self.clamp(forces[0], self.params.fmin, self.params.fmax)
        forces[1] = self.clamp(forces[1], self.params.fmin, self.params.fmax)
        forces[2] = self.clamp(forces[2], self.params.fmin, self.params.fmax)
        forces[3] = self.clamp(forces[3], self.params.fmin, self.params.fmax)

        return forces
    
    def determine_drag(self, state):
        import numpy as np
        # This function calculates the drag force acting on the drone assuming a
        # linear model of the drag proportional to speed.

        # Construct the Rotation Matrix from current state
        R1 = state[6:9]
        R2 = state[9:12]
        R3 = state[12:15]
        R = np.array([R1, R2, R3])
        
        #Transform the velocity from world frame into body frame
        vel_b = np.dot(np.linalg.inv(R) , [state[3], state[4], state[5]])
        
        #Calculates the drag force in the body frame, according to the linear
        #model
        fp = self.params.CD_bx * vel_b[0]
        fr = self.params.CD_by * vel_b[1]
        fy = self.params.CD_bz * vel_b[2]
        
        #Transform the forces into world frame, in x, y, and z direction
        drag = np.dot(R, [fp, fr, fy])
        return drag
    
    def innerdynamics_approx(self, old_state, forces, time):
        
        import numpy as np
        # This function takes in the current state values, the forces
        # applied by the motors, the quadrotor parameters and the time
        # to integrate over, and solves for the new state

        # NEW = OLD + delta_t*OLD_DERIV

        new_state = np.zeros(19)

        # Translational Velocities

        new_state[0] = old_state[0] + time*(old_state[3]) # x' = x'
        new_state[1] = old_state[1] + time*(old_state[4]) # y' = y'
        new_state[2] = old_state[2] + time*(old_state[5]) # z' = z'

        # Get the calculated drag force in x, y and z directions
        drag = self.determine_drag(old_state)

        # Translational Accelerations
        new_state[3] = old_state[3] + time*(sum(forces)*old_state[8] - drag[0])/self.params.m                # x'' = (f*R13 - fx)/m
        new_state[4] = old_state[4] + time*(sum(forces)*old_state[11] - drag[1])/self.params.m               # y'' = (f*R23 - fy)/m
        new_state[5] = old_state[5] + time*((sum(forces)*old_state[14] - drag[2])/self.params.m - self.params.g)  # z'' = (f*R33 - fz)/m - g

        # Change in Rotation Matrix
        new_state[6] = old_state[6] + time*(old_state[7]*old_state[17] - old_state[8]*old_state[16]) # R11' = R12*r - R13*q
        new_state[7] = old_state[7] + time*(old_state[8]*old_state[15] - old_state[6]*old_state[17]) # R12' = R13*p - R11*r
        new_state[8] = old_state[8] + time*(old_state[6]*old_state[16] - old_state[7]*old_state[15]) # R13' = R11*q - R12*p
        new_state[9] = old_state[9] + time*(old_state[10]*old_state[17] - old_state[11]*old_state[16]) # R21' = R22*r - R23*q
        new_state[10] = old_state[10] + time*(old_state[11]*old_state[15] - old_state[9]*old_state[17]) # R22' = R23*p - R21*r
        new_state[11] = old_state[11] + time*(old_state[9]*old_state[16] - old_state[10]*old_state[15]) # R23' = R21*q - R22*p
        new_state[12] = old_state[12] + time*(old_state[13]*old_state[17]- old_state[14]*old_state[16]) # R31' = R32*r - R33*q
        new_state[13] = old_state[13] + time*(old_state[14]*old_state[15] - old_state[12]*old_state[17]) # R32' = R33*p - R31*r
        new_state[14] = old_state[14] + time*(old_state[12]*old_state[16] - old_state[13]*old_state[15]) # R33' = R31*q - R32*p

        # p' = (1/Ix)*(L*(f2-f4) + (Iy-Iz)*r*q)
        new_state[15] = old_state[15] + time*((1/self.params.Ix)*(self.params.L*(forces[1]-forces[3]) + (self.params.Iy-self.params.Iz)*old_state[16]*old_state[17]))

        # q' = (1/Iy)*(L*(f3-f1) + (Iz-Ix)*r*p)
        new_state[16] = old_state[16] + time*((1/self.params.Iy)*(self.params.L*(forces[2]-forces[0]) + (self.params.Iz-self.params.Ix)*old_state[15]*old_state[17]))

        # r' = (1/Iz)*(K*(f1-f2+f3-f4) + (Ix-Iy)*p*q)
        new_state[17] = old_state[17] + time*((1/self.params.Iz)*(self.params.K*(forces[0]-forces[1]+forces[2]-forces[3]) + (self.params.Ix-self.params.Iy)*old_state[15]*old_state[16]))

        # z'' = (f*R33 - fz)/m - g
        new_state[18] = (sum(forces)*old_state[14] - drag[2])/self.params.m - self.params.g

        return new_state
    
    ############### MATLAB SIMLUATOR FUNCTIONS (ENDS HERE) ###############

    def dynamics_derivative(self, pitch, roll, z_vel, yaw_vel):
        """Return the state derivatives for the current state and input."""
        rates = self._inputs_to_desired_rates(pitch, roll, z_vel, yaw_vel)

        forces = self._determine_forces(*rates)

        return self._forces_to_derivatives(forces)

    def update_position(self, pitch, roll, z_vel, yaw_vel):
        """Compute the derivatives and integrate them based on inputs."""
        
        derivatives = self.dynamics_derivative(pitch, roll, z_vel, yaw_vel)
        self._integrate_derivatives(derivatives,
                                    self.params.inner_loop_cycle * 1e-6)
        
        
        #Uncomment the following and comment the code above for matlab simulator implementation
        '''
        rates = self._inputs_to_desired_rates(pitch, roll, z_vel, yaw_vel)
        forces = self._determine_forces(*rates)
        old_state = np.append(np.append(self.state.pos, self.state.vel), np.append(self.state.R, self.state.omega))
        new_state = self.innerdynamics_approx(old_state, forces, self.params.inner_loop_cycle * 1e-6)
        self.state.pos = np.array([new_state[0], new_state[1], new_state[2]])
        self.state.vel = np.array([new_state[3], new_state[4], new_state[5]])
        self.state.R = np.array([[new_state[6], new_state[7], new_state[8]], [new_state[9], new_state[10], new_state[11]], [new_state[12], new_state[13], new_state[14]]])
        self.state.omega = np.array([new_state[15], new_state[16], new_state[17]])
        '''
    def _inputs_to_desired_rates(self, pitch, roll, z_vel, yaw_vel):
        """Convert inputs to desired angular rates and thrust."""
        
        # Current roll, and yaw angles
        roll_cur, _, yaw_cur = self.state.rpy

        # r_des is simply the commanded yaw rate
        r_des = yaw_vel

        # calculate the commanded acceleration in the z direction,
        # (z_dot_des - z_dot) / tau_z
        z_ddot_des = (z_vel - self.state.vel[2]) / self.params.tau_Iz

        # And from this we may find the commanded thrust, (g + z_ddot_cmd)/R33
        c_des = (self.params.g + z_ddot_des) / self.state.R[2, 2]

        # Calculate the commanded yaw angle from:
        yaw_des = yaw_vel * self.params.tau_Iyaw + yaw_cur

        # R13_des = sin(yaw_des) * sin(roll_cmd)
        #         + cos(yaw_des) * cos(roll_cmd) * sin(pitch_cmd)
        r_13_des = (np.sin(yaw_des) * np.sin(roll) +
                    np.cos(yaw_des) * np.cos(roll) * np.sin(pitch))

        # R23_des = cos(roll_cmd) * sin(yaw_des) * sin(pitch_cmd)
        #         - cos(yaw_des) * sin(roll_cmd)
        r_23_des = (np.cos(roll) * np.sin(yaw_des) * np.sin(pitch) -
                    np.cos(yaw_des) * np.sin(roll))

        # p_des = (R21*(R13_des-R13) - R11*(R23_des-R23))/(R33*tau_rp)
        p_des = (self.state.R[1, 0] * (r_13_des - self.state.R[0, 2]) -
                 self.state.R[0, 0] * (r_23_des - self.state.R[1, 2]))
        p_des /= self.state.R[2, 2] * self.params.tau_rp

        # q_des = (R22*(R13_des-R13) - R12*(R23_des-R23))/(R33*tau_rp)
        q_des = (self.state.R[1, 1] * (r_13_des - self.state.R[0, 2]) -
                 self.state.R[0, 1] * (r_23_des - self.state.R[1, 2]))
        q_des /= self.state.R[2, 2] * self.params.tau_rp

        # Return everything!
        return p_des, q_des, r_des, c_des
        
        #Uncomment the following and comment the code above for matlab simulator implementation
        '''
        mat_state = self.convertstate_forward(self.state)
        return self.convertcmd(np.array([roll, pitch, yaw_vel, z_vel]), mat_state)
        '''
        
        
    def _determine_forces(self, p_des, q_des, r_des, c_des):  ## minor difference (a little bit more significant in x (stored in x~z2.png)), need more investigation
        """Convert desired angular rates and thrust to rotor forces."""
        L = self.params.L
        K = self.params.K
        m = self.params.m

        a = np.array(((0, L, 0, -L),
                      (-L, 0, L, 0),
                      (K, -K, K, -K),
                      (1 / m,  1 / m, 1 / m,  1 / m)),
                     dtype=np.float64)

        # The inertial matrix
        j = np.diag((self.params.Ix, self.params.Iy, self.params.Iz))

        # The current angular velocity vector
        omega = self.state.omega

        # The rate vector (our approximation of omega_dot)
        rate_vector = np.array(
            (((1 / self.params.tau_p) * (p_des - self.state.omega[0])),
             ((1 / self.params.tau_q) * (q_des - self.state.omega[1])),
             ((1 / self.params.tau_r) * (r_des - self.state.omega[2])))).T

        b = j.dot(rate_vector) + np.cross(omega, j.dot(omega))

        # Add c_des to the bottom of the row vector
        b = np.concatenate((b, [c_des]))

        # The rotor forces
        forces = np.linalg.solve(a, b)
        
        # Clamp the force between maxmium and minimum values
        forces[0] = self.clamp(forces[0], self.params.fmin, self.params.fmax)
        forces[1] = self.clamp(forces[1], self.params.fmin, self.params.fmax)
        forces[2] = self.clamp(forces[2], self.params.fmin, self.params.fmax)
        forces[3] = self.clamp(forces[3], self.params.fmin, self.params.fmax)
        
        # The rotor noise
        noise = np.random.normal(0, self.params.motor_noise, 4)
        
        # Return the four rotor forces
        return forces + noise
        
        #Uncomment the following and comment the code above for matlab simulator implementation
        '''
        mat_state = np.append(np.append(self.state.pos, self.state.vel), np.append(self.state.R, self.state.omega))
        return self.determineforces(mat_state, np.array([p_des, q_des, r_des, c_des]))
        '''
        
        
    def _forces_to_derivatives(self, forces):
        """Compute the state derivatives based on applied forces."""
        
        # Update position
        derivatives = quadrocopter_classes.State()

        derivatives.pos[:] = self.state.vel

        drag = self._compute_drag()

        # Update accelerations
        derivatives.acc = np.sum(forces) * self.state.R[:, 2] - drag

        # Add external forces
        for force in self.external_forces:
            derivatives.acc += force(self.state)

        # Normalize with mass and add gravity
        derivatives.acc /= self.params.m
        derivatives.acc[2] -= self.params.g

        # Update velocities
        derivatives.vel[:] = self.state.acc

        p, q, r = self.state.omega
        derivatives.R = self.state.R.dot(np.array([[0, -r, q],
                                                   [r, 0, -p],
                                                   [-q, p, 0]]))

        # Angular velocity changes
        f1, f2, f3, f4 = forces

        # p' = (1/Ix)*(L*(f2-f4) + (Iy-Iz)*r*q)
        p_dot = (self.params.L * (f2 - f4) +
                 (self.params.Iy - self.params.Iz) *
                 self.state.omega[2] * self.state.omega[1]) / self.params.Ix

        # q' = (1/Iy)*(L*(f3-f1) + (Iz-Ix)*r*p)
        q_dot = (self.params.L * (f3 - f1) +
                 (self.params.Iz - self.params.Ix) *
                 self.state.omega[2] * self.state.omega[0]) / self.params.Iy

        # r' = (1/Iz)*(K*(f1-f2+f3-f4) + (Ix-Iy)*p*q)
        r_dot = (self.params.K * (f1 - f2 + f3 - f4) +
                 (self.params.Ix - self.params.Iy) *
                 self.state.omega[0] * self.state.omega[1]) / self.params.Iz

        derivatives.omega = np.array([p_dot, q_dot, r_dot])

        return derivatives

    def _integrate_derivatives(self, derivatives, dt):
        """Simple euler integration to determine new states."""
        self.state.pos += dt * derivatives.pos
        self.state.vel += dt * derivatives.vel
        self.state.acc[:] = derivatives.acc
        
        self.state.R += dt * derivatives.R
        self.state.omega += dt * derivatives.omega

    def _compute_drag(self):
        """
        Computes velocities and applies linear drag model.

        Inverts the current rotation matrix and solves for the components of
        quadrocopter velocities in the body coordinates. Then a simple linear
        drag model equation is applied. This is done because the quadrocopter
        platform areas don't change in this refernce frame. The drag forces are
        returned in global coordinates.
        """

        v_b = np.linalg.solve(self.state.R, self.state.vel)

        drag_model = np.array((self.params.CD_bx,
                               self.params.CD_by,
                               self.params.CD_bz)) * v_b

        return self.state.R.dot(drag_model)


def wind_creator(direction, strength):
    """
    Return callable that computes the wind force on the quadrotor.

    Parameters:
    direction: 3d-array
        Direction vector for the wind.
    strength: float
        Strength of the wind in N / m^2
    """
    direction = np.asarray(direction, dtype=np.float).squeeze()
    direction /= np.linalg.norm(direction)

    quadrotor_length = 0.3
    quadrotor_height = 0.05

    norm_area = np.array((quadrotor_length * quadrotor_height,
                     quadrotor_length * quadrotor_height,
                     quadrotor_length ** 2))

    def wind_force(state):
        """
        Takes the quadrotor state and returns the wind force.

        Note:
        -----
        Homogeneous wind, this does not create any torques.
        """
        # Project surface areas into the wind direction
        area = np.abs(direction.dot(state.R)) * norm_area
        force = np.sum(area) * strength * direction
        return force

    return wind_force


def random_disturbance_creator(covariance, mean=None):
    """
    Add gaussian disturbance forces with a certain covariance function.

    Parameters
    ----------
    covariance: np.array
        A 3x3 array of the covariance matrix
    mean: np.array
        A 1d array of the 3 mean values (defaults to zero-mean)

    Returns
    -------
    disturbance: callable
        A function that can be used as an external force in quadsim
    """
    if mean is None:
        mean = np.zeros((3,))

    def random_force(state):
        """
        Takes the quadrotor state and returns the wind force.

        Parameters
        ----------
        state: quadrocopter_classes.State

        Returns
        -------
        force: np.array
        """
        return np.random.multivariate_normal(mean, covariance)

    return random_force
