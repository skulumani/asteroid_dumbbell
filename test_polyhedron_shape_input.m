% asteroid tester
clear all
clc

addpath(genpath(['.',filesep]));

load('/Users/shankar/Drive/GWU/research/asteroid_dumbbell/dynamics/CASTALIA/castalia_model.mat')

asteroid_params.F = F_32;
asteroid_params.V = V_32;
asteroid_params.G = 6.673e-20; % km^3/kg/sec^2
asteroid_params.sigma = 1.9;

asteroid_grav = polyhedron_shape_input(asteroid_params);


