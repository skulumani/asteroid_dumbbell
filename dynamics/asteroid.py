# Class file for the central asteroid
from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
import scipy.io
import os
import utilities
from point_cloud import wavefront
import pdb

# TODO: Implement the ability to input a filename for OBJ shape files
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
            else:
                print("Unknown asteroid. Use 'castalia', 'itokawa', or 'eros' only.")
            
            # reduce the number of faces/vertices somehow
            pdb.set_trace()
            ratio = num_faces / faces.shape[0]
            verts, faces = wavefront.decimate_numpy(verts, faces, ratio)

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

        # calculate all the edges - zero indexing for python
        Fa = F[:, 0] 
        Fb = F[:, 1]
        Fc = F[:, 2]

        V1 = V[Fa, :]
        V2 = V[Fb, :]
        V3 = V[Fc, :]

        # Get all edge vectors
        e1 = V2 - V1
        e2 = V3 - V2
        e3 = V1 - V3

        e1_vertex_map = np.vstack((Fb, Fa)).T
        e2_vertex_map = np.vstack((Fc, Fb)).T
        e3_vertex_map = np.vstack((Fa, Fc)).T

        # Normalize edge vectors
        # e1_norm=e1./repmat(sqrt(e1(:,1).^2+e1(:,2).^2+e1(:,3).^2),1,3);
        # e2_norm=e2./repmat(sqrt(e2(:,1).^2+e2(:,2).^2+e2(:,3).^2),1,3);
        # e3_norm=e3./repmat(sqrt(e3(:,1).^2+e3(:,2).^2+e3(:,3).^2),1,3);

        # normal to face
        normal_face = np.cross(e1, e2)
        normal_face = normal_face / \
            np.tile(np.reshape(
                np.sqrt(np.sum(normal_face**2, axis=1)), (num_f, 1)), (1, 3))

        # normal to each edge
        e1_normal = np.cross(e1, normal_face)
        e1_normal = e1_normal / \
            np.tile(np.reshape(
                np.sqrt(np.sum(e1_normal**2, axis=1)), (num_f, 1)), (1, 3))

        e2_normal = np.cross(e2, normal_face)
        e2_normal = e2_normal / \
            np.tile(np.reshape(
                np.sqrt(np.sum(e2_normal**2, axis=1)), (num_f, 1)), (1, 3))

        e3_normal = np.cross(e3, normal_face)
        e3_normal = e3_normal / \
            np.tile(np.reshape(
                np.sqrt(np.sum(e3_normal**2, axis=1)), (num_f, 1)), (1, 3))

        # calculate the center of each face
        center_face = 1.0 / 3 * (V[Fa, :] + V[Fb, :] + V[Fc, :])

        # Calculate Angle of face seen from vertices
        # Angle =  [acos(dot(e1_norm',-e3_norm'));acos(dot(e2_norm',-e1_norm'));acos(dot(e3_norm',-e2_norm'))]';

        # compute F dyad
        for ii in range(normal_face.shape[0]):
            F_face[:, :, ii] = np.outer(normal_face[ii, :], normal_face[ii, :])

        # loop over all the edges to figure out the common edges and calculate E_e

        # find common e1 edges
        e1_ind1b = utilities.ismember_index(-e1, e1)
        e1_ind2b = utilities.ismember_index(-e1, e2)
        e1_ind3b = utilities.ismember_index(-e1, e3)

        e2_ind1b = utilities.ismember_index(-e2, e1)
        e2_ind2b = utilities.ismember_index(-e2, e2)
        e2_ind3b = utilities.ismember_index(-e2, e3)

        e3_ind1b = utilities.ismember_index(-e3, e1)
        e3_ind2b = utilities.ismember_index(-e3, e2)
        e3_ind3b = utilities.ismember_index(-e3, e3)

        E1_edge = np.zeros([3, 3, num_f])
        E2_edge = np.zeros([3, 3, num_f])
        E3_edge = np.zeros([3, 3, num_f])

        for ii in range(num_f):
            # check each of the edges in the current face for a match in all the
            # other edges
            e1_face_map[ii, 0] = ii
            e2_face_map[ii, 0] = ii
            e3_face_map[ii, 0] = ii

            # e1 edge duplicate
            if e1_ind1b[ii] != invalid:
                e1_face_map[ii, 1] = e1_ind1b[ii]
            elif e1_ind2b[ii] != invalid:
                e1_face_map[ii, 2] = e1_ind2b[ii]
            elif e1_ind3b[ii] != invalid:
                e1_face_map[ii, 3] = e1_ind3b[ii]

            # e2 edge duplicate
            if e2_ind1b[ii] != invalid:
                e2_face_map[ii, 1] = e2_ind1b[ii]
            elif e2_ind2b[ii] != invalid:
                e2_face_map[ii, 2] = e2_ind2b[ii]
            elif e2_ind3b[ii] != invalid:
                e2_face_map[ii, 3] = e2_ind3b[ii]

            # e3 edge duplicate
            if e3_ind1b[ii] != invalid:
                e3_face_map[ii, 1] = e3_ind1b[ii]
            elif e3_ind2b[ii] != invalid:
                e3_face_map[ii, 2] = e3_ind2b[ii]
            elif e3_ind3b[ii] != invalid:
                e3_face_map[ii, 3] = e3_ind3b[ii]

            # find the edge normals for all edges of the current face
            # also pull out the edge normals for each adjacent face (3 adjacent
            # faces
            nA1 = e1_normal[e1_face_map[ii, 0], :]
            nA2 = e2_normal[e2_face_map[ii, 0], :]
            nA3 = e3_normal[e3_face_map[ii, 0], :]

            # find adjacent face for edge 1
            col = np.where(e1_face_map[ii, 1:] != invalid)[0][0]
            face_index = e1_face_map[ii, col + 1]

            if col == 0:  # adjacent face is also edge 1
                nB1 = e1_normal[face_index, :]
            elif col == 1:  # adjacent face is edge 2
                nB1 = e2_normal[face_index, :]
            elif col == 2:
                nB1 = e3_normal[face_index, :]

            nA = normal_face[ii, :]
            nB = normal_face[face_index, :]

            # second order dyadic tensor
            E1_edge[:, :, ii] = np.outer(nA, nA1) + np.outer(nB, nB1)

            # find adjacent face for edge 2
            col = np.where(e2_face_map[ii, 1:] != invalid)[0][0]
            face_index = e2_face_map[ii, col + 1]

            if col == 0:  # adjacent face is also edge 1
                nB2 = e1_normal[face_index, :]
            elif col == 1:  # adjacent face is edge 2
                nB2 = e2_normal[face_index, :]
            elif col == 2:
                nB2 = e3_normal[face_index, :]

            nB = normal_face[face_index, :]

            # second order dyadic tensor
            E2_edge[:, :, ii] = np.outer(nA, nA2) + np.outer(nB, nB2)

            # find adjacent face for edge 3
            col = np.where(e3_face_map[ii, 1:] != invalid)[0][0]
            face_index = e3_face_map[ii, col + 1]

            if col == 0:  # adjacent face is also edge 1
                nB3 = e1_normal[face_index, :]
            elif col == 1:  # adjacent face is edge 2
                nB3 = e2_normal[face_index, :]
            elif col == 2:
                nB3 = e3_normal[face_index, :]

            nB = normal_face[face_index, :]

            # second order dyadic tensor
            E3_edge[:, :, ii] = np.outer(nA, nA3) + np.outer(nB, nB3)

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
            U - gravitational potential
            U_grad
            U_grad_mat
            Ulaplace
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

        e1_lock = np.full(e1_face_map.shape, -1, dtype='int8')
        e2_lock = np.full(e2_face_map.shape, -1, dtype='int8')
        e3_lock = np.full(e3_face_map.shape, -1, dtype='int8')

        e1_vertex_map = self.asteroid_grav['e1_vertex_map']
        e2_vertex_map = self.asteroid_grav['e2_vertex_map']
        e3_vertex_map = self.asteroid_grav['e3_vertex_map']

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

        # vectorize w_face calculation
        ri = r_v[Fa, :]
        ri_norm = np.sqrt(np.sum(ri**2, axis=1))

        rj = r_v[Fb, :]
        rj_norm = np.sqrt(np.sum(rj**2, axis=1))

        rk = r_v[Fc, :]
        rk_norm = np.sqrt(np.sum(rk**2, axis=1))

        rjrk_cross = np.cross(rj, rk)
        num = np.sum(ri * rjrk_cross, axis=1)

        rjrk_dot = np.sum(rj * rk, axis=1)
        rkri_dot = np.sum(rk * ri, axis=1)
        rirj_dot = np.sum(ri * rj, axis=1)

        den = ri_norm * rj_norm * rk_norm + ri_norm * \
            rjrk_dot + rj_norm * rkri_dot + rk_norm * rirj_dot

        w_face = 2.0 * np.arctan2(num, den).reshape((num_f, 1))

        # check if point is inside or outside the body
        # zero when outside body and -G*sigma*4 pi on the inside
        inside_check = np.sum(w_face)

        if np.isclose(inside_check, 0):  # outside the body
            r1i = r_v[e1_vertex_map[:, 0], :]
            r1j = r_v[e1_vertex_map[:, 1], :]
            r1i_norm = np.sqrt(np.sum(r1i**2, axis=1))
            r1j_norm = np.sqrt(np.sum(r1j**2, axis=1))
            e1_norm = np.sqrt(np.sum(e1**2, axis=1))
            L1_edge = np.log((r1i_norm + r1j_norm + e1_norm) /
                             (r1i_norm + r1j_norm - e1_norm)).reshape((num_f, 1))

            r2i = r_v[e2_vertex_map[:, 0], :]
            r2j = r_v[e2_vertex_map[:, 1], :]
            r2i_norm = np.sqrt(np.sum(r2i**2, axis=1))
            r2j_norm = np.sqrt(np.sum(r2j**2, axis=1))
            e2_norm = np.sqrt(np.sum(e2**2, axis=1))
            L2_edge = np.log((r2i_norm + r2j_norm + e2_norm) /
                             (r2i_norm + r2j_norm - e2_norm)).reshape((num_f, 1))

            r3i = r_v[e3_vertex_map[:, 0], :]
            r3j = r_v[e3_vertex_map[:, 1], :]
            r3i_norm = np.sqrt(np.sum(r3i**2, axis=1))
            r3j_norm = np.sqrt(np.sum(r3j**2, axis=1))
            e3_norm = np.sqrt(np.sum(e3**2, axis=1))
            L3_edge = np.log((r3i_norm + r3j_norm + e3_norm) /
                             (r3i_norm + r3j_norm - e3_norm)).reshape((num_f, 1))

            # calculate the potential at input state
            U_edge = 0
            U_face = 0

            U_grad_edge = np.zeros(3)
            U_grad_face = np.zeros(3)

            U_grad_mat_edge = np.zeros((3, 3))
            U_grad_mat_face = np.zeros((3, 3))

            # sum over edges
            for ii in range(num_f):

                # face contribution
                # this can potentially be done completely outside of the for loop
                U_face = U_face + \
                    r_v[Fa[ii], :].dot(F_face[:, :, ii]).dot(
                        r_v[Fa[ii], :].T) * w_face[ii, 0]
                U_grad_face = U_grad_face + \
                    F_face[:, :, ii].dot(r_v[Fa[ii], :].T) * w_face[ii, 0]
                U_grad_mat_face = U_grad_mat_face + \
                    F_face[:, :, ii] * w_face[ii]

                # compute contributions for the three edges on this face but ignore if
                # it's a duplicate

                # edge 1
                if np.sum(e1_lock[ii, :]) == -4:  # new edge

                    U1 = r_v[e1_vertex_map[ii, 0], :].dot(E1_edge[:, :, ii]).dot(
                        r_v[e1_vertex_map[ii, 0], :].T) * L1_edge[ii, 0]
                    U1_grad = E1_edge[:, :, ii].dot(
                        r_v[e1_vertex_map[ii, 1], :].T) * L1_edge[ii, 0]
                    U1_grad_mat = E1_edge[:, :, ii] * L1_edge[ii, 0]

                    col = np.where(e1_face_map[ii, 1:] != -1)[0][0]
                    row = e1_face_map[ii, col + 1]

                    e1_lock[ii, 0] = ii
                    e1_lock[ii, col + 1] = row
                    # update lock
                    if col == 0:  # adjacent face is also edge 1
                        e1_lock[row, 1] = ii
                    elif col == 1:  # adjacent face is edge 2
                        e2_lock[row, 1] = ii
                    elif col == 2:
                        e3_lock[row, 1] = ii

                else:
                    e1_lock[ii, 0] = ii

                    U1 = 0
                    U1_grad = np.zeros(3)
                    U1_grad_mat = np.zeros((3, 3))

                # edge 2
                if np.sum(e2_lock[ii, :]) == -4:
                    U2 = r_v[e2_vertex_map[ii, 0], :].dot(E2_edge[:, :, ii]).dot(
                        r_v[e2_vertex_map[ii, 0], :].T) * L2_edge[ii, 0]
                    U2_grad = E2_edge[:, :, ii].dot(
                        r_v[e2_vertex_map[ii, 0], :].T) * L2_edge[ii, 0]
                    U2_grad_mat = E2_edge[:, :, ii] * L2_edge[ii, 0]

                    col = np.where(e2_face_map[ii, 1:] != -1)[0][0]
                    row = e2_face_map[ii, col + 1]

                    e2_lock[ii, 0] = ii
                    e2_lock[ii, col + 1] = row

                    # update lock
                    if col == 0:  # duplicate edge is edge 1 on another face
                        e1_lock[row, 2] = ii
                    elif col == 1:  # adjacent face is edge 2
                        e2_lock[row, 2] = ii
                    elif col == 2:
                        e3_lock[row, 2] = ii

                else:
                    e2_lock[ii, 0] = ii

                    U2 = 0
                    U2_grad = np.zeros(3)
                    U2_grad_mat = np.zeros((3, 3))

                # edge 3
                if np.sum(e3_lock[ii, :]) == -4:
                    U3 = r_v[e3_vertex_map[ii, 0], :].dot(E3_edge[:, :, ii]).dot(
                        r_v[e3_vertex_map[ii, 0], :].T) * L3_edge[ii, 0]
                    U3_grad = E3_edge[:, :, ii].dot(
                        r_v[e3_vertex_map[ii, 0], :].T) * L3_edge[ii, 0]
                    U3_grad_mat = E3_edge[:, :, ii] * L3_edge[ii, 0]

                    col = np.where(e3_face_map[ii, 1:] != -1)[0][0]
                    row = e3_face_map[ii, col + 1]

                    e3_lock[ii, 0] = ii
                    e3_lock[ii, col + 1] = row
                    # update lock
                    if col == 0:  # duplicate is in e1
                        e1_lock[row, 3] = ii
                    elif col == 1:
                        e2_lock[row, 3] = ii
                    elif col == 2:
                        e3_lock[row, 3] = ii

                else:
                    e3_lock[ii, 0] = ii

                    U3 = 0
                    U3_grad = np.zeros(3)
                    U3_grad_mat = np.zeros((3, 3))

                U_edge = U_edge + U1 + U2 + U3
                U_grad_edge = U_grad_edge + U1_grad.reshape(U_grad_edge.shape) + U2_grad.reshape(
                    U_grad_edge.shape) + U3_grad.reshape(U_grad_edge.shape)
                U_grad_mat_edge = U_grad_mat_edge + U1_grad_mat + U2_grad_mat + U3_grad_mat

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
