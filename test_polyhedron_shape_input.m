% asteroid tester
clear all
clc

addpath(genpath(['.',filesep]));

load('./dynamics/CASTALIA/castalia_model.mat')
% load('./dynamics/ITOKAWA/itokawa_model.mat')
asteroid_params.F = F_512;
asteroid_params.V = V_512;
asteroid_params.G = 6.673e-20; % km^3/kg/sec^2
% asteroid_params.sigma = 2.1; % castalia
asteroid_params.sigma = 1.9; % itokawa

asteroid_grav = polyhedron_shape_input(asteroid_params);

state = [1;0.2;0];

num_runs = 1;
time = zeros(num_runs,1);
for ii = 1:num_runs
    start = tic;
    [U_mat,Ug_mat,Ug_mat_mat, Ulap_mat] = polyhedron_potential(state, asteroid_grav);
    time(ii) = toc(start);
end

fprintf('%g loops, best of 3:  %4.2f ms per loop\n', num_runs,min(time)*1e3)
