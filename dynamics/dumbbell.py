import numpy as np
import scipy.linalg

import kinematics.attitude as attitude
import pdb

class Dumbbell(object):
    """Dumbbell spacecraft model

    """
    def __init__(self, m1=100.0, m2=100.0, l=0.003):
        """Initialize the dumbbell instance with all of it's properties
            
        """

        self.m1 = m1 # kg first mass
        self.m2 = m2 # kg second mass
        self.l = l # km rigid link
        self.r1 = 0.001 # km radius of each spherical mass 
        self.r2 = 0.001

        self.mratio = self.m2/(self.m1+self.m2)
        self.lcg1 = self.mratio*self.l; # distance from m1 to the CG along the b1hat direction
        self.lcg2 = self.l - self.lcg1

        self.zeta1 = np.array([-self.lcg1,0,0])
        self.zeta2 = np.array([self.lcg2,0,0])

        self.Jm1 = 2.0/5*self.m1*self.r1**2 * np.diag([1,1,1])
        self.Jm2 = 2.0/5*self.m2*self.r2**2 * np.diag([1,1,1])

        self.J = self.Jm1 + self.Jm2 + self.m1 *(np.inner(self.zeta1,self.zeta1)*np.eye(3,3) - np.outer(self.zeta1,self.zeta1)) + self.m2 * (np.inner(self.zeta2,self.zeta2)*np.eye(3,3) - np.outer(self.zeta2,self.zeta2))
        self.Jd = self.m1*np.outer(self.zeta1,self.zeta1) + self.m2*np.outer(self.zeta2,self.zeta2) + self.Jm1/2 + self.Jm2/2

        # controller parameters
        OS_translation = 5/100
        Tp_translation = 5
        Ts_translation = 10 

        OS_rotation = 5/100
        Tp_rotation = 5
        Ts_rotation = 10

        self.zeta_translation = - np.log(OS_translation) / np.sqrt(np.pi**2 + np.log(OS_translation)**2)
        self.wn_translation = 4 / self.zeta_translation / Ts_translation

        self.zeta_rotation = - np.log(OS_rotation) / np.sqrt(np.pi**2 + np.log(OS_rotation)**2)
        self.wn_rotation = 4 / self.zeta_rotation / Ts_rotation

        self.kR = self.wn_rotation**2 
        self.kW = 2 * self.zeta_rotation * self.wn_rotation 
        
        self.kx =  (self.m1 + self.m2) * self.wn_translation**2
        self.kv = (self.m1 + self.m2) * 2 * self.zeta_translation * self.wn_translation

    def eoms_inertial_ode(self, t, state, ast):
        """Inertial dumbbell equations of motion about an asteroid
        
        Inputs:
            t - 
            state -
            ast - Asteroid class object holding the asteroid gravitational model and
            other useful parameters
        """
        # unpack the state
        pos = state[0:3] # location of the center of mass in the inertial frame
        vel = state[3:6] # vel of com in inertial frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
        ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame

        Ra = attitude.rot3(ast.omega*t, 'c') # asteroid body frame to inertial frame

        # unpack parameters for the dumbbell
        J = self.J

        rho1 = self.zeta1
        rho2 = self.zeta2

        # position of each mass in the asteroid frame
        z1 = Ra.T.dot(pos + R.dot(rho1))
        z2 = Ra.T.dot(pos + R.dot(rho2))

        z = Ra.T.dot(pos) # position of COM in asteroid frame

        # compute the potential at this state
        (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
        (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

        F1 = self.m1*Ra.dot(U1_grad)
        F2 = self.m2*Ra.dot(U2_grad)

        M1 = self.m1 * attitude.hat_map(rho1).dot(R.T.dot(Ra).dot(U1_grad))
        M2 = self.m2 * attitude.hat_map(rho2).dot(R.T.dot(Ra).dot(U2_grad))
        # M1 = np.zeros(3)
        # M2 = np.zeros_like(3)

        pos_dot = vel
        vel_dot = 1/(self.m1+self.m2) *(F1 + F2)
        R_dot = R.dot(attitude.hat_map(ang_vel)).reshape(9)
        ang_vel_dot = np.linalg.inv(J).dot(-np.cross(ang_vel,J.dot(ang_vel)) + M1 + M2)

        statedot = np.hstack((pos_dot, vel_dot, R_dot, ang_vel_dot))

        return statedot
    def eoms_inertial(self, state,t, ast):
        """Inertial dumbbell equations of motion about an asteroid
        
        Inputs:
            t - 
            state -
            ast - Asteroid class object holding the asteroid gravitational model and
            other useful parameters
        """
        # unpack the state
        pos = state[0:3] # location of the center of mass in the inertial frame
        vel = state[3:6] # vel of com in inertial frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
        ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame

        Ra = attitude.rot3(ast.omega*t, 'c') # asteroid body frame to inertial frame

        # unpack parameters for the dumbbell
        J = self.J

        rho1 = self.zeta1
        rho2 = self.zeta2

        # position of each mass in the asteroid frame
        z1 = Ra.T.dot(pos + R.dot(rho1))
        z2 = Ra.T.dot(pos + R.dot(rho2))

        z = Ra.T.dot(pos) # position of COM in asteroid frame

        # compute the potential at this state
        (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
        (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

        F1 = self.m1*Ra.dot(U1_grad)
        F2 = self.m2*Ra.dot(U2_grad)

        M1 = self.m1 * attitude.hat_map(rho1).dot(R.T.dot(Ra).dot(U1_grad))
        M2 = self.m2 * attitude.hat_map(rho2).dot(R.T.dot(Ra).dot(U2_grad))
        # M1 = np.zeros(3)
        # M2 = np.zeros_like(3)

        pos_dot = vel
        vel_dot = 1/(self.m1+self.m2) *(F1 + F2)
        R_dot = R.dot(attitude.hat_map(ang_vel)).reshape(9)
        ang_vel_dot = np.linalg.inv(J).dot(-np.cross(ang_vel,J.dot(ang_vel)) + M1 + M2)

        statedot = np.hstack((pos_dot, vel_dot, R_dot, ang_vel_dot))

        return statedot

    def eoms_relative(self, state, t, ast):
        """Relative EOMS defined in the rotating asteroid frame

        This function defines the motion of a dumbbell spacecraft in orbit around an asteroid.
        The EOMS are defined relative to the asteroid itself, which is in a state of constant rotation.
        You need to use this function with scipy.integrate.odeint

        Inputs:
            t - current time of simulation (sec)
            state - (18,) relative state of dumbbell with respect to asteroid
                pos - state[0:3] in km position of the dumbbell with respect to the asteroid and defined in the asteroid fixed frame
                vel - state[3:6] in km/sec is the velocity of dumbbell wrt the asteroid and defined in the asteroid fixed frame
                R - state[6:15] rotation matrix which converts vectors from the dumbbell frame to the asteroid frame
                w - state[15:18] rad/sec angular velocity of the dumbbell wrt inertial frame and defined in the asteroid frame
            ast - asteroid object

        Output:
            state_dot - (18,) derivative of state. The order is the same as the input state.
        """
        
        # unpack the state
        pos = state[0:3] # location of the COM of dumbbell in asteroid fixed frame
        vel = state[3:6] # vel of com wrt to asteroid expressed in the asteroid fixed frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to asteroid body frame R = R_A^T R_1
        w = state[15:18] # angular velocity of sc wrt inertial frame and expressed in asteroid fixed frame

        Ra = attitude.rot3(ast.omega*t, 'c') # asteroid body frame to inertial frame

        # unpack parameters for the dumbbell
        m1 = self.m1
        m2 = self.m2
        m = m1 + m2
        J = self.J
        Jr = R.dot(J).dot(R.T)
        Wa = ast.omega*np.array([0,0,1]) # angular velocity vector of asteroid

        # the position of each mass in the dumbbell body frame
        rho1 = self.zeta1
        rho2 = self.zeta2

        # position of each mass in the asteroid frame
        z1 = pos + R.dot(rho1)
        z2 = pos + R.dot(rho2)

        z = pos # position of COM in asteroid frame

        # compute the potential at this state
        (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
        (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

        # force due to each mass expressed in asteroid body frame
        F1 = m1*U1_grad
        F2 = m2*U2_grad

        M1 = m1*attitude.hat_map(R.dot(rho1)).dot(U1_grad) 
        M2 = m2*attitude.hat_map(R.dot(rho2)).dot(U2_grad) 

        # M1 = np.zeros(3)
        # M2 = np.zeros_like(M1)
        # state derivatives
        pos_dot = vel - attitude.hat_map(Wa).dot(pos)
        vel_dot = 1/m * (F1 + F2 - m * attitude.hat_map(Wa).dot(vel))
        # vel_dot = 1/m * (F_com) 
        R_dot = attitude.hat_map(w).dot(R) - attitude.hat_map(Wa).dot(R)
        R_dot = R_dot.reshape(9)
        w_dot = np.linalg.inv(Jr).dot(M1 + M2 - Jr.dot(attitude.hat_map(Wa)).dot(w))
        state_dot = np.hstack((pos_dot, vel_dot, R_dot, w_dot))
        
        return state_dot

    def eoms_hamilton_relative_ode(self, t, state, ast):
        """Hamiltonian form of Relative EOMS defined in the rotating asteroid frame

        This function defines the motion of a dumbbell spacecraft in orbit around an asteroid.
        The EOMS are defined relative to the asteroid itself, which is in a state of constant rotation.
        You need to use this function with scipy.integrate.odeint

        Inputs:
            t - current time of simulation (sec)
            state - (18,) relative hamiltonian state of dumbbell with respect to asteroid
                pos - state[0:3] in km position of the dumbbell with respect to the asteroid and defined in the asteroid fixed frame
                lin_mom - state[3:6] in kg km/sec is the linear momentum of dumbbell wrt the asteroid and defined in the asteroid fixed frame
                R - state[6:15] rotation matrix which converts vectors from the dumbbell frame to the asteroid frame
                ang_mom - state[15:18] J rad/sec angular momentum of the dumbbell wrt inertial frame and defined in the asteroid frame
            ast - asteroid object

        Output:
            state_dot - (18,) derivative of state. The order is the same as the input state.
        """
        
        # unpack the state
        pos = state[0:3] # location of the COM of dumbbell in asteroid fixed frame
        lin_mom = state[3:6] # lin_mom of com wrt to asteroid expressed in the asteroid fixed frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to asteroid body frame R = R_A^T R_1
        ang_mom = state[15:18] # angular momentum of sc wrt inertial frame and expressed in asteroid fixed frame
        
        Ra = attitude.rot3(ast.omega*t, 'c') # asteroid body frame to inertial frame

        # unpack parameters for the dumbbell
        m1 = self.m1
        m2 = self.m2
        m = m1 + m2
        J = self.J
        Jr = R.dot(J).dot(R.T)
        Wa = ast.omega*np.array([0,0,1]) # angular velocity vector of asteroid

        # the position of each mass in the dumbbell body frame
        rho1 = self.zeta1
        rho2 = self.zeta2

        # position of each mass in the asteroid frame
        z1 = pos + R.dot(rho1)
        z2 = pos + R.dot(rho2)

        z = pos # position of COM in asteroid frame

        # compute the potential at this state
        (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
        (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

        # force due to each mass expressed in asteroid body frame
        F1 = m1*U1_grad
        F2 = m2*U2_grad

        M1 = m1*attitude.hat_map(R.dot(rho1)).dot(U1_grad) 
        M2 = m2*attitude.hat_map(R.dot(rho2)).dot(U2_grad) 

        vel = lin_mom / (m1 + m2)
        w = np.linalg.inv(Jr).dot(ang_mom)
        # state derivatives
        pos_dot = vel - attitude.hat_map(Wa).dot(pos)
        lin_mom_dot = F1 + F2 - attitude.hat_map(Wa).dot(lin_mom)
        R_dot = attitude.hat_map(w).dot(R) - attitude.hat_map(Wa).dot(R)
        R_dot = R_dot.reshape(9)
        ang_mom_dot = M1 + M2  - attitude.hat_map(Wa).dot(ang_mom) 
        state_dot = np.hstack((pos_dot, lin_mom_dot, R_dot, ang_mom_dot))
        
        return state_dot
    def eoms_hamilton_relative(self, state, t, ast):
        """Hamiltonian form of Relative EOMS defined in the rotating asteroid frame

        This function defines the motion of a dumbbell spacecraft in orbit around an asteroid.
        The EOMS are defined relative to the asteroid itself, which is in a state of constant rotation.
        You need to use this function with scipy.integrate.odeint

        Inputs:
            t - current time of simulation (sec)
            state - (18,) relative hamiltonian state of dumbbell with respect to asteroid
                pos - state[0:3] in km position of the dumbbell with respect to the asteroid and defined in the asteroid fixed frame
                lin_mom - state[3:6] in kg km/sec is the linear momentum of dumbbell wrt the asteroid and defined in the asteroid fixed frame
                R - state[6:15] rotation matrix which converts vectors from the dumbbell frame to the asteroid frame
                ang_mom - state[15:18] J rad/sec angular momentum of the dumbbell wrt inertial frame and defined in the asteroid frame
            ast - asteroid object

        Output:
            state_dot - (18,) derivative of state. The order is the same as the input state.
        """
        
        # unpack the state
        pos = state[0:3] # location of the COM of dumbbell in asteroid fixed frame
        lin_mom = state[3:6] # lin_mom of com wrt to asteroid expressed in the asteroid fixed frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to asteroid body frame R = R_A^T R_1
        ang_mom = state[15:18] # angular momentum of sc wrt inertial frame and expressed in asteroid fixed frame
        
        Ra = attitude.rot3(ast.omega*t, 'c') # asteroid body frame to inertial frame

        # unpack parameters for the dumbbell
        m1 = self.m1
        m2 = self.m2
        m = m1 + m2
        J = self.J
        Jr = R.dot(J).dot(R.T)
        Wa = ast.omega*np.array([0,0,1]) # angular velocity vector of asteroid

        # the position of each mass in the dumbbell body frame
        rho1 = self.zeta1
        rho2 = self.zeta2

        # position of each mass in the asteroid frame
        z1 = pos + R.dot(rho1)
        z2 = pos + R.dot(rho2)

        z = pos # position of COM in asteroid frame

        # compute the potential at this state
        (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
        (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

        # force due to each mass expressed in asteroid body frame
        F1 = m1*U1_grad
        F2 = m2*U2_grad

        M1 = m1*attitude.hat_map(R.dot(rho1)).dot(U1_grad) 
        M2 = m2*attitude.hat_map(R.dot(rho2)).dot(U2_grad) 

        vel = lin_mom / (m1 + m2)
        w = np.linalg.inv(Jr).dot(ang_mom)
        # state derivatives
        pos_dot = vel - attitude.hat_map(Wa).dot(pos)
        lin_mom_dot = F1 + F2 - attitude.hat_map(Wa).dot(lin_mom)
        R_dot = attitude.hat_map(w).dot(R) - attitude.hat_map(Wa).dot(R)
        R_dot = R_dot.reshape(9)
        ang_mom_dot = M1 + M2  - attitude.hat_map(Wa).dot(ang_mom) 
        state_dot = np.hstack((pos_dot, lin_mom_dot, R_dot, ang_mom_dot))
        
        return state_dot
    def eoms_inertial_control(self, state,t, ast):
        """Inertial dumbbell equations of motion about an asteroid with body fixed control capability
        
        Inputs:
            t - Current simulation time step
            state - (18,) array which defines the state of the vehicle 
                pos - (3,) position of the dumbbell with respect to the asteroid center of mass
                    and expressed in the inertial frame
                vel - (3,) velocity of the dumbbell with respect to the asteroid center of mass
                    and expressed in the inertial frame
                R - (9,) attitude of the dumbbell with defines the transformation of a vector in the
                    dumbbell frame to the inertial frame
                ang_vel - (3,) angular velocity of the dumbbell with respect to the inertial frame 
                    and represented in the dumbbell frame
            ast - Asteroid class object holding the asteroid gravitational model and
                other useful parameters
        """
        # unpack the state
        pos = state[0:3] # location of the center of mass in the inertial frame
        vel = state[3:6] # vel of com in inertial frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
        ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame

        Ra = attitude.rot3(ast.omega*t, 'c') # asteroid body frame to inertial frame

        # unpack parameters for the dumbbell
        J = self.J

        rho1 = self.zeta1
        rho2 = self.zeta2

        # position of each mass in the asteroid frame
        z1 = Ra.T.dot(pos + R.dot(rho1))
        z2 = Ra.T.dot(pos + R.dot(rho2))

        z = Ra.T.dot(pos) # position of COM in asteroid frame

        # compute the potential at this state
        (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
        (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

        F1 = self.m1*Ra.dot(U1_grad)
        F2 = self.m2*Ra.dot(U2_grad)

        M1 = self.m1 * attitude.hat_map(rho1).dot(R.T.dot(Ra).dot(U1_grad))
        M2 = self.m2 * attitude.hat_map(rho2).dot(R.T.dot(Ra).dot(U2_grad))

        # compute the control input
        u_m = self.attitude_controller(t, state, M1+M2)
        u_f = self.translation_controller(t, state, F1+F2)

        pos_dot = vel
        vel_dot = 1/(self.m1+self.m2) *(F1 + F2 + u_f)
        R_dot = R.dot(attitude.hat_map(ang_vel)).reshape(9)
        ang_vel_dot = np.linalg.inv(J).dot(-np.cross(ang_vel,J.dot(ang_vel)) + M1 + M2 + u_m)

        statedot = np.hstack((pos_dot, vel_dot, R_dot, ang_vel_dot))

        return statedot

    def inertial_energy(self,time,state, ast):
        """Compute the kinetic and potential energy of the dumbbell given the current state
        
        Input:
            vel - nx3 velocity in inertial frame (km/sec)
            ang_vel - nx3 angular velocity of dumbbell frame wrt to inertial frame expressed in dumbbell frame (rad/sec)

        Outputs:
            T - nx1 kinetic energy array which should be the same length as state input

        """
        # some constants
        m = self.m1 + self.m2 # total mass of dumbbell in kg
        Jd = self.Jd

        KE = np.zeros(time.shape[0])
        PE = np.zeros(time.shape[0])

        for ii in range(time.shape[0]):
            pos = state[ii,0:3] # location of the center of mass in the inertial frame
            vel = state[ii,3:6] # vel of com in inertial frame
            R = np.reshape(state[ii,6:15],(3,3)) # sc body frame to inertial frame
            ang_vel = state[ii,15:18] # angular velocity of sc wrt inertial frame defined in body frame

            Ra = attitude.rot3(ast.omega*time[ii], 'c') # asteroid body frame to inertial frame

            # position of each mass in the inertial frame
            z1 = Ra.T.dot(pos + R.dot(self.zeta1))
            z2 = Ra.T.dot(pos + R.dot(self.zeta2))

            (U1, _, _, _) = ast.polyhedron_potential(z1)
            (U2, _, _, _) = ast.polyhedron_potential(z2)

            PE[ii] = -self.m1*U1 - self.m2*U2
            KE[ii] = 1.0/2 * m * vel.dot(vel) + 1.0/2 * np.trace(attitude.hat_map(ang_vel).dot(Jd).dot(attitude.hat_map(ang_vel).T))

        return KE, PE

    def relative_energy(self, time, state, ast):
        """Compute the KE and PE of the relative equations of motion

        """

        m = self.m1 + self.m2
        J1 = self.J
        Jd = self.Jd

        KE = np.zeros(time.shape[0])
        PE = np.zeros(time.shape[0])

        for ii in range(time.shape[0]):
            pos = state[ii, 0:3]  # location of the COM wrt asteroid in the asteroid frame
            vel = state[ii, 3:6]  # vel of COM wrt asteroid in asteroid frame
            R = np.reshape(state[ii, 6:15], (3, 3))  # sc body frame to asteroid frame
            ang_vel = state[ii, 15:18]  # angular velocity of sc wrt inertial frame defined in asteroid frame

            # position of each mass in the inertial frame
            z1 = pos + R.dot(self.zeta1)
            z2 = pos + R.dot(self.zeta2)

            (U1, _, _, _) = ast.polyhedron_potential(z1)
            (U2, _, _, _) = ast.polyhedron_potential(z2)

            Jdr = R.dot(Jd).dot(R.T)

            PE[ii] = -self.m1 * U1 - self.m2 * U2
            KE[ii] = 1/2 * m * vel.dot(vel) + 1/2 * np.trace(attitude.hat_map(ang_vel).dot(Jdr).dot(attitude.hat_map(ang_vel).T))

        return KE, PE

    def attitude_controller(self, time, state, ext_moment):
        """SE(3) Attitude Controller
        
        This function will return the control input to track a desired attitude trajectory


        """
        # extract the state
        pos = state[0:3] # location of the center of mass in the inertial frame
        vel = state[3:6] # vel of com in inertial frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
        ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame
        # compute the desired attitude command
        Rd, Rd_dot, ang_vel_d, ang_vel_d_dot = self.desired_attitude(time)
        # determine error between command and current state
        eR = 1/2 * attitude.vee_map(Rd.T.dot(R) - R.T.dot(Rd))
        eW = ang_vel - R.T.dot(Rd).dot(ang_vel_d)
        # compute attitude input
        u_m = (-self.kR*eR - self.kW*eW + np.cross(ang_vel, self.J.dot(ang_vel)) 
                - self.J.dot( attitude.hat_map(ang_vel).dot(R.T).dot(Rd).dot(ang_vel_d)-
                    R.T.dot(Rd).dot(ang_vel_d_dot)) - ext_moment)
        return u_m

    def translation_controller(self, time, state, ext_force):
        """SE(3) Translational Controller

        Inputs:

        Outputs:
            u_f - force command in the dumbbell frame

        """

        # extract the state
        pos = state[0:3] # location of the center of mass in the inertial frame
        vel = state[3:6] # vel of com in inertial frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
        ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame
      
        m = self.m1 + self.m2

        # figure out the desired trajectory
        x_des, xd_des, xdd_des = self.desired_translation(time)

        # compute the error
        ex = pos - x_des
        ev = vel - xd_des
        # compute the control
        u_f = - self.kx * ex - self.kv * ev - ext_force + m * xdd_des

        return u_f

    def desired_attitude(self, time, alpha=2*np.pi/500, axis=np.array([0, 1, 0])):
        """Desired attitude trajectory

        This function will output a desired attitude trajectory. The controller will use this 
        trajectory in it's computations. The outputs will be the desired attitude matrix and
        the desired angular velocity:

        Outputs:
            Rd_sc2int - 3x3 array defining the transformation from the spacecraft frame to the
                inertial frame
            w_sc2int - 3 array defining the angular velocity of the spacecraft frame with respect
                to the inertial frame and defined in the spacecraft fixed frame

        """
        Rd = scipy.linalg.expm(alpha * time * attitude.hat_map(axis) ) 
        Rd_dot = alpha * attitude.hat_map(axis).dot(
                    scipy.linalg.expm(alpha * attitude.hat_map(axis)))

        ang_vel_d = attitude.vee_map(Rd.T.dot(Rd_dot))
        ang_vel_d_dot = np.zeros_like(ang_vel_d)

        return (Rd, Rd_dot, ang_vel_d, ang_vel_d_dot) 

    def desired_translation(self, time):
        """Desired translational trajectory

        This function will output the desired translational states, namely the desired position and
        velocity. This position and velocity will be defined in the inertial reference frame.

        """
        x_des = np.array([1, 1, 1])
        xd_des = np.array([0, 0, 0])
        xdd_des = np.array([0, 0, 0])

        return (x_des, xd_des, xdd_des)
