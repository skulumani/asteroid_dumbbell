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

    def eoms_intertial(t, state, ast):
        """Inertial dumbbell equations of motion about an asteroid
        
        Inputs:
            t - 
            state -
            ast - Asteroid class object holding the asteroid gravitational model and
            other useful parameters
        """

        pass

    def eoms_relative(t,state,ast):

        pass