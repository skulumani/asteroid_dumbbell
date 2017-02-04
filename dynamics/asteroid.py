# Class file for the central asteroid
import numpy as np
import scipy.io
import os

class Asteroid(object):
    """An asteroid that we're orbiting about

    Attributes:
        G - gravitational constant km^3/kg/sec^2
        M - mass in kg
        sigma - density in grams/cm^3
        mu - M*G gravitational parameter km^3/sec^2
        axes - semi axis lengths in meters
        Ixx - Principle moment of inertia kg m^2
        Iyy - 
        Izz -
        omega - rotation period rad/sec
        mass_param - mass parameter
        res_radius - resonance radius
        dist_scale - distance scale for plotting
        time_scale - time scaling factor
        C20 - spherical harmonic coefficient
        C22 - spherical harmoning coefficient
    """
    G = 6.673e-20

    def __init__(self, name, num_faces):
        """Initialize the asteroid instance with it's properties

        """
        dir_path = os.path.dirname(os.path.realpath(__file__))

        self.name = name

        if name == 'castalia':
            self.M = 1.4091e12
            self.sigma = 2.1
            self.axes = np.array([1.6130, 0.9810, 0.8260])*1.0e3 / 2.0
            self.omega = 2*np.pi/4.07/3600
            
            # self.C20 = -7.275e-2
            # self.C22 = 2.984e-2
            
            mat = scipy.io.loadmat(dir_path + "/CASTALIA/castalia_model.mat")

        elif name == 'itokawa':
            self.M = 3.51e10
            self.sigma = 1.9
            self.axes = np.array([535, 294, 209]) # size in meters
            self.omega = 2*np.pi/12.132/3600
            
            mat = scipy.io.loadmat(dir_path + "/ITOKAWA/itokawa_model.mat")
        else:
            print("Unknown asteroid. Use 'castalia or 'itokawa' only.")

        if int(num_faces) in [2**j for j in range(5,12+1)]:
            F_key = "F_" + str(num_faces)
            V_key = "V_" + str(num_faces)
        else:
            print("That number of faces is not possible.")

        self.F = mat[F_key]
        self.V = mat[V_key]

        print("Using %g faces for %s." % (self.F.shape[0],self.name))
        print("Polyhedron Model: %g faces, %g vertices." % (self.F.shape[0],self.V.shape[0]))

        self.mu = self.G*self.M

        # Compute some inertia properties
        self.Ixx = self.M/5*(self.axes[1]**2+self.axes[2]**2)
        self.Iyy = self.M/5*(self.axes[0]**2+self.axes[2]**2)
        self.Izz = self.M/5*(self.axes[0]**2+self.axes[1]**2)

        self.mass_param = (self.Iyy-self.Ixx)/(self.Izz-self.Ixx)
        self.res_radius = (self.mu/self.omega**2)**(1.0/3)
        self.dist_scale = self.res_radius
        self.time_scale = self.omega
        self.C20 = -1.0/2*(self.Izz-self.Ixx)*(2-self.mass_param)/self.dist_scale**2/self.M
        self.C22 = 1.0/4*(self.Izz-self.Ixx)*self.mass_param/self.dist_scale**2/self.M
        # calculate the distance
        self.r = np.sqrt(self.V[:,0]**2 + self.V[:,1]**2 + self.V[:,2]**2)
        self.long = np.arctan2(self.V[:,1],self.V[:,0]) * 180/np.pi
        self.lat = np.arcsin(self.V[:,2]/self.r) ** 180/np.pi
        
        # sort in order of increasing radius
        index = np.argsort(self.r)
        self.r = self.r[index]
        self.long = self.long[index]
        self.lat = self.lat[index]
        
        # compute a bunch of parameters for the polyhedron model
        # () = self.polyhedron_shape_input(self)

    def polyhedron_shape_input(self):
        """Precomputes parameters for a given polyhedron shape model
        
        Adds attributes to the class - polyhedron gravity model
        """


