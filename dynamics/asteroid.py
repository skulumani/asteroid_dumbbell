# Class file for the central asteroid
import numpy as np
import scipy.io
import os
import utilities
import pdb

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
        a = self.polyhedron_shape_input()

    def polyhedron_shape_input(self):
        """Precomputes parameters for a given polyhedron shape model
        
        Adds attributes to the class - polyhedron gravity model
        """
        G = self.G
        sigma = self.sigma/1000*(100/1)**3*(1000/1)**3 # kg/km^3
        F = self.F
        V = self.V

        num_v = V.shape[0]
        num_f = F.shape[0]
        num_e = 3*(num_v-2)

        # calculate shape parameters
        # calculate vectors for each edge (loop through F and difference the
        # vectors) then store them
        e1_face_map = np.zeros([num_f,4])
        e2_face_map = np.zeros([num_f,4])
        e3_face_map = np.zeros([num_f,4])

        F_face = np.zeros([3,3,num_f])

        # calculate all the edges - zero indexing for python
        Fa = F[:,0]-1
        Fb = F[:,1]-1
        Fc = F[:,2]-1
        
        V1 = V[Fa,:]
        V2 = V[Fb,:]
        V3 = V[Fc,:]

        # Get all edge vectors
        e1 = V[Fb,:]-V[Fa,:]
        e2 = V[Fc,:]-V[Fb,:]
        e3 = V[Fa,:]-V[Fc,:]

        e1_vertex_map = np.vstack((Fb, Fa))
        e2_vertex_map = np.vstack((Fc, Fb))
        e3_vertex_map = np.vstack((Fa, Fc))

        # Normalize edge vectors
        # e1_norm=e1./repmat(sqrt(e1(:,1).^2+e1(:,2).^2+e1(:,3).^2),1,3); 
        # e2_norm=e2./repmat(sqrt(e2(:,1).^2+e2(:,2).^2+e2(:,3).^2),1,3); 
        # e3_norm=e3./repmat(sqrt(e3(:,1).^2+e3(:,2).^2+e3(:,3).^2),1,3);
        
        # normal to face
        normal_face = np.cross(e1,e2)
        normal_face = normal_face / np.tile(np.reshape(np.sqrt(np.sum(normal_face**2,axis=1)),(num_f,1)),(1,3))

        # normal to each edge
        e1_normal = np.cross(e1,normal_face) 
        e1_normal = e1_normal / np.tile(np.reshape(np.sqrt(np.sum(e1_normal**2,axis=1)),(num_f,1)),(1,3))

        e2_normal = np.cross(e2,normal_face)
        e2_normal = e2_normal / np.tile(np.reshape(np.sqrt(np.sum(e2_normal**2,axis=1)),(num_f,1)),(1,3))

        e3_normal = np.cross(e3,normal_face)
        e3_normal = e3_normal / np.tile(np.reshape(np.sqrt(np.sum(e3_normal**2,axis=1)),(num_f,1)),(1,3))

        # calculate the center of each face
        center_face = 1.0/3 * (V[Fa,:] + V[Fb,:] + V[Fc,:])

        # Calculate Angle of face seen from vertices
        # Angle =  [acos(dot(e1_norm',-e3_norm'));acos(dot(e2_norm',-e1_norm'));acos(dot(e3_norm',-e2_norm'))]';

        # compute F dyad
        for ii in range(normal_face.shape[0]):
            F_face[:,:,ii] = np.outer(normal_face[ii,:],normal_face[ii,:])
        
        # loop over all the edges to figure out the common edges and calculate E_e

        # find common e1 edges
        [e1_ind1a,e1_ind1b] = ismember(-e1,e1,'rows');
        [e1_ind2a,e1_ind2b] = ismember(-e1,e2,'rows');
        [e1_ind3a,e1_ind3b] = ismember(-e1,e3,'rows');

        [e2_ind1a,e2_ind1b] = ismember(-e2,e1,'rows');
        [e2_ind2a,e2_ind2b] = ismember(-e2,e2,'rows');
        [e2_ind3a,e2_ind3b] = ismember(-e2,e3,'rows');

        [e3_ind1a,e3_ind1b] = ismember(-e3,e1,'rows');
        [e3_ind2a,e3_ind2b] = ismember(-e3,e2,'rows');
        [e3_ind3a,e3_ind3b] = ismember(-e3,e3,'rows');

        E1_edge = zeros(3,3,num_f);
        E2_edge = zeros(3,3,num_f);
        E3_edge = zeros(3,3,num_f);

        for ii = 1:num_f

        % check each of the edges in the current face for a match in all the
        % other edges
        e1_face_map(ii,1) = ii;
        e2_face_map(ii,1) = ii;
        e3_face_map(ii,1) = ii;

        % e1 edge duplicate

        ind1 = e1_ind1b(ii);
        ind2 = e1_ind2b(ii);
        ind3 = e1_ind3b(ii);

        if ind1 ~= 0
        e1_face_map(ii,2) = ind1;
        elseif ind2 ~= 0
        e1_face_map(ii,3) = ind2;
        elseif ind3 ~= 0
        e1_face_map(ii,4) = ind3;
        end

        % e2 edge duplicate

        ind1 = e2_ind1b(ii);
        ind2 = e2_ind2b(ii);
        ind3 = e2_ind3b(ii);

        if ind1 ~= 0
        e2_face_map(ii,2) = ind1;
        elseif ind2 ~= 0
        e2_face_map(ii,3) = ind2;
        elseif ind3 ~= 0
        e2_face_map(ii,4) = ind3;
        end

        % e3 edge duplicate

        ind1 = e3_ind1b(ii);
        ind2 = e3_ind2b(ii);
        ind3 = e3_ind3b(ii);

        if ind1 ~= 0
        e3_face_map(ii,2) = ind1;
        elseif ind2 ~= 0
        e3_face_map(ii,3) = ind2;
        elseif ind3 ~= 0
        e3_face_map(ii,4) = ind3;
        end

        % find the edge normals for all edges of the current face
        % also pull out the edge normals for each adjacent face (3 adjacent
        % faces
        nA1 = e1_normal(e1_face_map(ii,1),:);
        nA2 = e2_normal(e2_face_map(ii,1),:);
        nA3 = e3_normal(e3_face_map(ii,1),:);

        % find adjacent face for edge 1
        col = find(e1_face_map(ii,2:end) ~= 0);
        face_index = e1_face_map(ii,col+1);

        if col == 1 % adjacent face is also edge 1

        nB1 = e1_normal(face_index,:); 
        elseif col == 2 % adjacent face is edge 2
        nB1 = e2_normal(face_index,:); 
        elseif col == 3
        nB1 = e3_normal(face_index,:); 
        end

        nA = normal_face(ii,:);
        nB = normal_face(face_index,:);

        E1_edge(:,:,ii) = nA'*nA1 + nB'*nB1; % second order dyadic tensor

        % find adjacent face for edge 2
        col = find(e2_face_map(ii,2:end) ~= 0);
        face_index = e2_face_map(ii,col+1);

        if col == 1 % adjacent face is also edge 1
        nB2 = e1_normal(face_index,:); 
        elseif col == 2 % adjacent face is edge 2
        nB2 = e2_normal(face_index,:); 
        elseif col == 3
        nB2 = e3_normal(face_index,:); 
        end


        nB = normal_face(face_index,:);

        E2_edge(:,:,ii) = nA'*nA2 + nB'*nB2; % second order dyadic tensor

        % find adjacent face for edge 3
        col = find(e3_face_map(ii,2:end) ~= 0);
        face_index = e3_face_map(ii,col+1);

        if col == 1 % adjacent face is also edge 1
        nB3 = e1_normal(face_index,:); 
        elseif col == 2 % adjacent face is edge 2
        nB3 = e2_normal(face_index,:); 
        elseif col == 3
        nB3 = e3_normal(face_index,:); 
        end


        nB = normal_face(face_index,:);

        E3_edge(:,:,ii) = nA'*nA3 + nB'*nB3; % second order dyadic tensor

        end

        % save as a structure with all the precomputed polyhedron potential data

        asteroid_grav.F = F;
        asteroid_grav.V = V;

        asteroid_grav.V1 = V1;
        asteroid_grav.V2 = V2;
        asteroid_grav.V3 = V3;

        asteroid_grav.e1 = e1;
        asteroid_grav.e2 = e2;
        asteroid_grav.e3 = e3;

        asteroid_grav.e1_face_map = e1_face_map;
        asteroid_grav.e2_face_map = e2_face_map;
        asteroid_grav.e3_face_map = e3_face_map;

        asteroid_grav.e1_vertex_map = e1_vertex_map;
        asteroid_grav.e2_vertex_map = e2_vertex_map;
        asteroid_grav.e3_vertex_map = e3_vertex_map;

        asteroid_grav.normal_face = normal_face;
        asteroid_grav.center_face = center_face;

        asteroid_grav.e1_normal = e1_normal;
        asteroid_grav.e2_normal = e2_normal;
        asteroid_grav.e3_normal = e3_normal;

        asteroid_grav.E1_edge = E1_edge;
        asteroid_grav.E2_edge = E2_edge;
        asteroid_grav.E3_edge = E3_edge;

        asteroid_grav.F_face = F_face;

        asteroid_grav.num_f = num_f;
        asteroid_grav.num_e = num_e;
        asteroid_grav.num_v = num_v;

        asteroid_grav.G = G;
        asteroid_grav.sigma = sigma;

