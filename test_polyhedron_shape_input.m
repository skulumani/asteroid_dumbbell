% asteroid tester
clear all
clc

addpath(genpath(['.',filesep]));

load('./dynamics/CASTALIA/castalia_model.mat')

asteroid_params.F = F_32;
asteroid_params.V = V_32;
asteroid_params.G = 6.673e-20; % km^3/kg/sec^2
asteroid_params.sigma = 2.1;

asteroid_grav = polyhedron_shape_input(asteroid_params);

state = [1;0.2;0];

[U_mat,Ug_mat,Ug_mat_mat, Ulap_mat] = polyhedron_potential(state, asteroid_grav);