import numpy as np
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

        # M1 = self.m1 * np.cross(Ra.T.dot(rho1),R.T.dot(U1_grad))
        # M2 = self.m2 * np.cross(Ra.T.dot(rho2),R.T.dot(U2_grad))
        # M1 = np.zeros(3)
        # M2 = M1
        
        M1 = self.m1 * np.cross(rho1, R.T.dot(Ra).dot(U1_grad))
        M2 = self.m2 * np.cross(rho2, R.T.dot(Ra).dot(U2_grad))

        pos_dot = vel
        vel_dot = 1/(self.m1+self.m2) *(F1 + F2)
        R_dot = R.dot(attitude.hat_map(ang_vel)).reshape(9)
        ang_vel_dot = np.linalg.inv(J).dot(-np.cross(ang_vel,J.dot(ang_vel)) + M1 + M2)

        statedot = np.hstack((pos_dot, vel_dot, R_dot, ang_vel_dot))

        return statedot

    def eoms_relative(self, state, t, ast):
        """Relative EOMS - motion of dumbbell wrt to asteroid expressed in asteroid fixed frame

        Inputs:
            t - current time of simulation (sec)
            state - relative state of dumbbell with respect to asteroid
            ast - asteroid object

        Output:

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
        Omega = ast.omega*np.array([0,0,1]) # angular velocity vector of asteroid

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
        # F_com = (m1+m2)*U_grad_com

        # compute the moments due to each mass
        # pdb.set_trace()
        # M1 = m1 * np.cross(U1_grad, R.dot(rho1))
        # M2 = m2 * np.cross(U2_grad, R.dot(rho2))
        M1 = m1 * np.cross(rho1, R.T.dot(U1_grad))
        M2 = m2 * np.cross(rho2, R.T.dot(U2_grad))
        # M1 = np.zeros(3)
        # M2 = M1
        
        # state derivatives
        pos_dot = vel - attitude.hat_map(Omega).dot(pos)
        vel_dot = 1/m * (F1 + F2 - m * attitude.hat_map(Omega).dot(vel))
        # vel_dot = 1/m * (F_com) 
        R_dot = attitude.hat_map(w).dot(R) - attitude.hat_map(Omega).dot(R)
        R_dot = R_dot.reshape(9)
        w_dot = np.linalg.inv(Jr).dot(M1 + M2 - attitude.hat_map(Omega).dot(Jr).dot(w) 
                      + attitude.hat_map(w).dot(Jr).dot(w) - Jr.dot(attitude.hat_map(Omega)).dot(w) 
                      + attitude.hat_map(Omega).dot(Jr).dot(w) - attitude.hat_map(w).dot(Jr).dot(w))

        state_dot = np.hstack((pos_dot,vel_dot,R_dot,w_dot))
        
        return state_dot

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
