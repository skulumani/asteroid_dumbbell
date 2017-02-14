import numpy as np
import attitude_ref.attitude as attitude
import pdb

class Dumbbell():
    """Dumbbell spacecraft model

    """
    def __init__(self):
        """Initialize the dumbbell instance with all of it's properties
            
        """

        self.m1 = 100 # kg first mass
        self.m2 = 100 # kg second mass
        self.l = 0.003 # km rigid link
        self.r1 = 0.001 # km radius of each spherical mass 
        self.r2 = 0.001

        self.lcg1 = self.m2/(self.m1+self.m2)*self.l; # distance from m1 to the CG along the b1hat direction
        self.lcg2 = self.l - self.lcg1

        self.zeta1 = np.array([-self.lcg1,0,0])
        self.zeta2 = np.array([self.lcg2,0,0])

        self.Jm1 = 2/5*self.m1*self.r1**2 * np.diag([1,1,1])
        self.Jm2 = 2/5*self.m2*self.r2**2 * np.diag([1,1,1])

        self.J = self.Jm1 + self.Jm2 + self.m1 *(np.inner(self.zeta1,self.zeta1)*np.eye(3,3) - np.outer(self.zeta1,self.zeta1)) + self.m2 * (np.inner(self.zeta2,self.zeta2)*np.eye(3,3) - np.outer(self.zeta2,self.zeta2))

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

        Ra = attitude.rot3(-ast.omega*t) # asteroid body frame to inertial frame

        # unpack parameters for the dumbbell
        J = self.J

        rho1 = self.zeta1
        rho2 = self.zeta2

        # position of each mass in the inertial frame
        z1 = Ra.T.dot(pos + R.dot(rho1))
        z2 = Ra.T.dot(pos + R.dot(rho2))

        z = Ra.T.dot(pos) # position of COM in asteroid frame

        # compute the potential at this state
        (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
        (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

        F1 = self.m1*Ra.dot(U1_grad)
        F2 = self.m2*Ra.dot(U2_grad)

        M1 = self.m1 * attitude.vee_map(np.outer(R.T.dot(U1_grad), Ra.T.dot(rho1)) - np.outer(Ra.T.dot(rho1), U1_grad.dot(R)))
        M2 = self.m2 * attitude.vee_map(np.outer(R.T.dot(U2_grad), Ra.T.dot(rho2)) - np.outer(Ra.T.dot(rho2), U2_grad.dot(R)))

        pos_dot = vel
        vel_dot = 1/(self.m1+self.m2) *(F1 + F2)
        R_dot = R.dot(attitude.hat_map(ang_vel)).reshape(9)
        ang_vel_dot = np.linalg.inv(J).dot(-np.cross(ang_vel,J.dot(ang_vel)) + M1 + M2)

        statedot = np.hstack((pos_dot,vel_dot,R_dot,ang_vel_dot))

        return statedot

    def eoms_relative(self,t,state,ast):

        pass

    def inertial_kinetic_energy(self,state):
        """Compute the kinetic energy of the dumbbell given the current state
        
        Input:
            state - nx18 state array
                pos - 0:3 position in inertial frame (km)
                vel - 3:6 velocity in inertial frame (km/sec)
                R - 6:16 rotation matrix from dumbbell frame to inertial frame
                ang_vel - 16:18 angular velocity of dumbbell frame wrt to inertial frame expressed in dumbbell frame (rad/sec)

        Outputs:
            T - nx1 kinetic energy array which should be the same length as state input

        """

        