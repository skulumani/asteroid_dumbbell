% asteroid tester

addpath(genpath(['.',filesep]));

load('/Users/shankar/Drive/GWU/research/asteroid_dumbbell/dynamics/CASTALIA/castalia_model.mat')

asteroid_params.F = F_32;
asteroid_params.V = V_32;
asteroid_params.G = 6.673e-20; % km^3/kg/sec^2
asteroid_params.sigma = 1.9;

asteroid_grav = polyhedron_shape_input(asteroid_params);

asteroid_grav.F 
asteroid_grav.V 

asteroid_grav.V1
asteroid_grav.V2
asteroid_grav.V3

asteroid_grav.e1
asteroid_grav.e2
asteroid_grav.e3

asteroid_grav.e1_face_map 
asteroid_grav.e2_face_map 
asteroid_grav.e3_face_map 

asteroid_grav.e1_vertex_map 
asteroid_grav.e2_vertex_map 
asteroid_grav.e3_vertex_map 

asteroid_grav.normal_face 
asteroid_grav.center_face 

asteroid_grav.e1_normal 
asteroid_grav.e2_normal 
asteroid_grav.e3_normal

asteroid_grav.E1_edge 
asteroid_grav.E2_edge 
asteroid_grav.E3_edge 

asteroid_grav.F_face 

asteroid_grav.num_f 
asteroid_grav.num_e 
asteroid_grav.num_v 

asteroid_grav.G 
asteroid_grav.sigma 
