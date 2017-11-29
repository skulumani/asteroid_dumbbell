# Class file for the central asteroid
from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
import scipy.io
import os
import utilities
from point_cloud import wavefront, polyhedron
import pdb

# TODO: Implement the ability to input a filename for OBJ shape files
# TODO: Need to add some tests to check if the shape model is a closed polyhedron
#   Can check to make sure each edge is a member of exactly 2 faces (for triangular meshes)
# TODO: Implement a Cython/C++ version of both of these methods
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
    # TODO: Deprecate matlab matfile
    def __init__(self, name, num_faces, shape_flag='mat'):
        """Initialize the asteroid instance with it's properties

        """
        dir_path = os.path.dirname(os.path.realpath(__file__))

        self.name = name
        # either use the matlab file or read the OBJ file
        if shape_flag == 'mat':  # use matlab shape data
            if name == 'castalia':

                mat = scipy.io.loadmat(
                    dir_path + "/CASTALIA/castalia_model.mat")

            elif name == 'itokawa':
                mat = scipy.io.loadmat(dir_path + "/ITOKAWA/itokawa_model.mat")
            else:
                print("Unknown asteroid. Use 'castalia or 'itokawa' only.")

            if num_faces in [4092, 2048, 1024, 512, 256, 128, 64, 32]:
                F_key = "F_" + str(num_faces)
                V_key = "V_" + str(num_faces)
            else:
                print("That number of faces is not possible.")
                return 1

            self.F = mat[F_key] - 1
            self.V = mat[V_key]

        elif shape_flag == 'obj':  # read directly from the OBJ file
            if name == 'castalia':
                verts, faces = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj')
            elif name == 'itokawa':
                verts, faces = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')
            elif name == 'eros':
                verts, faces = wavefront.read_obj('./data/shape_model/EROS/eros_low.obj')
            elif name == 'cube':
                verts, faces = wavefront.read_obj('./integration/cube.obj')
                # translate so center of object is at origin
                verts = verts - np.array([0.5, 0.5, 0.5])
            else:
                print("Unknown asteroid. Use 'castalia', 'itokawa', or 'eros' only.")
            
            # reduce the number of faces/vertices somehow
            # if (num_faces != 0 and num_faces < faces.shape[0]):
            #     ratio = 1 - num_faces / faces.shape[0]
            #     verts, faces = wavefront.decimate_numpy(verts, faces, ratio)

            self.F = faces
            self.V = verts

            # print("Using %g faces for %s." % (self.F.shape[0],self.name))
            # print("Polyhedron Model: %g faces, %g vertices." % (self.F.shape[0],self.V.shape[0]))
        
        # define the mass properties of the asteroid
        if name == 'castalia':
            self.M = 1.4091e12
            self.sigma = 2.1  # g/cm^3
            self.axes = np.array([1.6130, 0.9810, 0.8260]) / 2.0
            self.omega = 2 * np.pi / 4.07 / 3600

            # self.C20 = -7.275e-2
            # self.C22 = 2.984e-2
        elif name == 'itokawa':
            self.M = 3.51e10
            self.sigma = 1.9  # g/cm^3
            self.axes = np.array([535, 294, 209]) / 1.0e3  # size in meters
            self.omega = 2 * np.pi / 12.132 / 3600

        elif name == 'eros':
            self.M = 4.463e-4 / self.G
            self.sigma = 2.67 # g/cm^3
            self.axes = np.array([34.4, 11.7, 11.7])  # size in kilometers
            self.omega = 2 * np.pi / 5.27 / 3600
        elif name == 'cube':
            self.M = 1
            self.sigma = 1
            self.axes=np.array([0.9, 1.0, 1.1])
            self.omega = 1
        else:
            print("Unknown asteroid.")

        self.mu = self.G * self.M
        self.sigma = self.sigma / 1000 * \
            (100 / 1)**3 * (1000 / 1)**3  # kg/km^3

        # Compute some inertia properties
        self.Ixx = self.M / 5 * (self.axes[1]**2 + self.axes[2]**2)
        self.Iyy = self.M / 5 * (self.axes[0]**2 + self.axes[2]**2)
        self.Izz = self.M / 5 * (self.axes[0]**2 + self.axes[1]**2)

        self.mass_param = (self.Iyy - self.Ixx) / (self.Izz - self.Ixx)
        self.res_radius = (self.mu / self.omega**2)**(1.0 / 3)
        self.dist_scale = self.res_radius
        self.time_scale = self.omega
        self.C20 = -1.0 / 2 * (self.Izz - self.Ixx) * \
            (2 - self.mass_param) / self.dist_scale**2 / self.M
        self.C22 = 1.0 / 4 * (self.Izz - self.Ixx) * \
            self.mass_param / self.dist_scale**2 / self.M
        # calculate the distance
        self.r = np.sqrt(self.V[:, 0]**2 + self.V[:, 1]**2 + self.V[:, 2]**2)
        self.long = np.arctan2(self.V[:, 1], self.V[:, 0]) * 180 / np.pi
        self.lat = np.arcsin(self.V[:, 2] / self.r) ** 180 / np.pi

        # sort in order of increasing radius
        index = np.argsort(self.r)
        self.r = self.r[index]
        self.long = self.long[index]
        self.lat = self.lat[index]

        # compute a bunch of parameters for the polyhedron model
        self.asteroid_grav = self.polyhedron_shape_input()
    
    # TODO: Add documentation
    def polyhedron_shape_input(self):
        """Precomputes parameters for a given polyhedron shape model

        Adds attributes to the class - polyhedron gravity model
        """
        print("Computing polyhedron shape parameters")

        invalid = -1

        G = self.G
        sigma = self.sigma
        F = self.F
        V = self.V

        num_v = V.shape[0]
        num_f = F.shape[0]
        num_e = 3 * (num_v - 2)
        
        # calculate shape parameters
        # calculate vectors for each edge (loop through F and difference the
        # vectors) then store them
        e1_face_map = np.full([num_f, 4], invalid, dtype='int')
        e2_face_map = np.full([num_f, 4], invalid, dtype='int')
        e3_face_map = np.full([num_f, 4], invalid, dtype='int')

        F_face = np.zeros([3, 3, num_f])
        
        # TODO Make this a named tuple or dictionary
        (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3, e1_vertex_map, e2_vertex_map,
        e3_vertex_map, normal_face, e1_normal, e2_normal,e3_normal, center_face, e_vertex_map, unique_index) = wavefront.polyhedron_parameters(V, F)

        # compute F dyad
        F_face = np.einsum('ij,ik->jki', normal_face, normal_face)
        # loop over all the edges to figure out the common edges and calculate E_e
        # find common e1 edges

        (e1_ind1b, e1_ind2b, e1_ind3b,
        e2_ind1b, e2_ind2b, e2_ind3b,
        e3_ind1b, e3_ind2b, e3_ind3b) = wavefront.search_edge_vertex_map(e1_vertex_map,
                                                                         e2_vertex_map, 
                                                                         e3_vertex_map)
        # build the edge face maps
        e1_face_map, e2_face_map, e3_face_map = wavefront.build_edge_face_map(e1_ind1b, e1_ind2b, e1_ind3b,
                                                                              e2_ind1b, e2_ind2b, e2_ind3b,
                                                                              e3_ind1b, e3_ind2b, e3_ind3b)
        
        # adjacent faces for edges
        E1_edge, E2_edge, E3_edge = wavefront.compute_edge_dyad(e1_face_map, e2_face_map, e3_face_map,
                                    e1_normal, e2_normal, e3_normal,
                                    normal_face)

        # save as a structure with all the precomputed polyhedron potential data
        asteroid_grav = {
            'F':                F,
            'V':                V,
            'V1':               V1,
            'V2':               V2,
            'V3':               V3,
            'e1':               e1,
            'e2':               e2,
            'e3':               e3,
            'e1_face_map':      e1_face_map,
            'e2_face_map':      e2_face_map,
            'e3_face_map':      e3_face_map,
            'e1_vertex_map':    e1_vertex_map,
            'e2_vertex_map':    e2_vertex_map,
            'e3_vertex_map':    e3_vertex_map,
            'e_vertex_map':     e_vertex_map,
            'unique_index':     unique_index,
            'e1_ind1b':         e1_ind1b,
            'e1_ind2b':         e1_ind2b,
            'e1_ind3b':         e1_ind3b,
            'e2_ind1b':         e2_ind1b,
            'e2_ind2b':         e2_ind2b,
            'e2_ind3b':         e2_ind3b,
            'e3_ind1b':         e3_ind1b,
            'e3_ind2b':         e3_ind2b,
            'e3_ind3b':         e3_ind3b,
            'normal_face':      normal_face,
            'center_face':      center_face,
            'e1_normal':        e1_normal,
            'e2_normal':        e2_normal,
            'e3_normal':        e3_normal,
            'E1_edge':          E1_edge,
            'E2_edge':          E2_edge,
            'E3_edge':          E3_edge,
            'F_face':           F_face,
            'num_f':            num_f,
            'num_v':            num_v,
            'num_e':            num_e,
            'Fa':               Fa,
            'Fb':               Fb,
            'Fc':               Fc}

        return asteroid_grav

    def polyhedron_potential(self, state):
        """Polyhedron Potential Model 

        Inputs: 
            self - the asteroid class that has all the polyhedron potential properties
            state - 3x1 position vector in the asteroid body fixed frame in km

        Outputs:
            U - gravitational potential - distance**2 / time**2
            U_grad - gravitational attraction - distance / time**2
            U_grad_mat - gravitational gradient matrix
            Ulaplace - laplacian
        """
        F = self.F
        V = self.V

        Fa = self.asteroid_grav['Fa']
        Fb = self.asteroid_grav['Fb']
        Fc = self.asteroid_grav['Fc']

        e1 = self.asteroid_grav['e1']
        e2 = self.asteroid_grav['e2']
        e3 = self.asteroid_grav['e3']

        e1_face_map = self.asteroid_grav['e1_face_map']
        e2_face_map = self.asteroid_grav['e2_face_map']
        e3_face_map = self.asteroid_grav['e3_face_map']


        e1_vertex_map = self.asteroid_grav['e1_vertex_map']
        e2_vertex_map = self.asteroid_grav['e2_vertex_map']
        e3_vertex_map = self.asteroid_grav['e3_vertex_map']
        
        e_vertex_map = self.asteroid_grav['e_vertex_map']
        unique_index = self.asteroid_grav['unique_index']

        e1_ind1b = self.asteroid_grav['e1_ind1b']
        e1_ind2b = self.asteroid_grav['e1_ind2b']
        e1_ind3b = self.asteroid_grav['e1_ind3b']

        e2_ind1b = self.asteroid_grav['e2_ind1b']
        e2_ind2b = self.asteroid_grav['e2_ind2b']
        e2_ind3b = self.asteroid_grav['e2_ind3b']

        e3_ind1b = self.asteroid_grav['e3_ind1b']
        e3_ind2b = self.asteroid_grav['e3_ind2b']
        e3_ind3b = self.asteroid_grav['e3_ind3b']

        normal_face = self.asteroid_grav['normal_face']

        e1_normal = self.asteroid_grav['e1_normal']
        e2_normal = self.asteroid_grav['e2_normal']
        e3_normal = self.asteroid_grav['e3_normal']

        E1_edge = self.asteroid_grav['E1_edge']
        E2_edge = self.asteroid_grav['E2_edge']
        E3_edge = self.asteroid_grav['E3_edge']

        F_face = self.asteroid_grav['F_face']

        num_f = self.asteroid_grav['num_f']
        num_e = self.asteroid_grav['num_e']
        num_v = self.asteroid_grav['num_v']

        G = self.G
        sigma = self.sigma
        
        # distance from state to each vertex
        r_v = V - np.tile(state, (num_v, 1))

        # function to compute laplacia factor (wf)
        w_face = polyhedron.laplacian_factor(r_v, Fa, Fb, Fc)

        # check if point is inside or outside the body
        # zero when outside body and -G*sigma*4 pi on the inside
        inside_check = np.sum(w_face)
        if np.isclose(inside_check, 0):  # outside the body
            L1_edge, L2_edge, L3_edge = polyhedron.edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map)
            
            # compute the contribution of the each face
            U_face, U_grad_face, U_grad_mat_face  = polyhedron.face_contribution(r_v, Fa, F_face, w_face)
            U_edge, U_grad_edge, U_grad_mat_edge = polyhedron.edge_contribution(state, e_vertex_map, unique_index, V, E1_edge, E2_edge, E3_edge, L1_edge, L2_edge, L3_edge)

            # combine edge and face summations
            U = 1 / 2 * G * sigma * U_edge - 1 / 2 * G * sigma * U_face
            U_grad = -G * sigma * U_grad_edge + G * sigma * U_grad_face
            U_grad_mat = G * sigma * U_grad_mat_edge - G * sigma * U_grad_mat_face
            Ulaplace = -G * sigma * np.sum(w_face)
        else:
            U = 0
            U_grad = np.zeros(3)
            U_grad_mat = np.zeros((3, 3))
            Ulaplace = 0
        #     print("INSIDE ASTEROID!")

        return (U, U_grad, U_grad_mat, Ulaplace)

