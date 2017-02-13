import numpy as np
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

    def eoms_intertial(self, t, state, ast):
        """Inertial dumbbell equations of motion about an asteroid
        
        Inputs:
            t - 
            state -
            ast - Asteroid class object holding the asteroid gravitational model and
            other useful parameters
        """
        pdb.set_trace()
        # unpack the state
        pos = state[0:2] # location of the center of mass in the inertial frame
        vel = state[3:5] # vel of com in inertial frame
        R = np.reshape(state[6:14],(3,3)) # sc body frame to inertial frame
        w = state[15:17] # angular velocity of sc wrt inertial frame defined in body frame

        Ra = ROT3(-ast.omega*t) # asteroid body frame to inertial frame

        # unpack parameters for the dumbbell
        J = self.J

        zeta1 = self.zeta1
        zeta2 = self.zeta2

        # position of each mass in the inertial frame
        z1 = Ra.T*(pos + R * zeta1)
        z2 = Ra.T*(pos + R * zeta2)

        z = Ra.T * pos # position of COM in asteroid frame

        statedot = np.zeros(state.shape)

        return statedot

    def eoms_relative(t,state,ast):

        pass