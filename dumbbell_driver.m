% Driver function to simulate dumbbell about an asteroid
clc
clearvars
close all

% start_time = tic;

%% ASTEROID CONSTANTS
fprintf('LOADING CONSTANTS\n')
addpath(genpath(['.',filesep]));

constants = load_constants('castalia','true'); % only 1024 faces
asteroid_grav = polyhedron_shape_input(constants);
constants.asteroid_grav = asteroid_grav;

constants.pot_model = 'polyhedron'; % or mascon or matlab
constants.ode_options = odeset('RelTol',1e-9,'AbsTol',1e-9);

%% SPACECRAFT CONSTANTS

constants.m1 = 100; % kg first mass
constants.m2 = 100; % kg second mass
constants.l = 0.002; % m rigid link
constants.lcg = constants.m2/(constants.m1+constants.m2)*constants.l;
constants.It = constants.m1*constants.lcg^2+constants.m2*(constants.l-constants.lcg)^2;
constants.Ia = 2/5*constants.m1*0.001^2 + 2/5*constants.m2*0.001^2;
constants.J = diag([constants.Ia,constants.It,constants.It]);

%% SIMULATE ODE
fprintf('SIMULATING!\n')

num_steps = 100000;
constants.num_steps = num_steps;
t_final = 1000;

tspan = linspace(0,t_final,constants.num_steps);
initial_pos = [1.495746722510590;0.000001002669660;0.006129720493607]; % km for center of mass
initial_vel = [0.000000302161724;-0.000899607989820;-0.000000013286327];
initial_R = reshape(eye(3,3),9,1);
initial_W = [0;0;0];

initial_state = [initial_pos;initial_vel;initial_R;initial_W];
[t,state_body] = ode113(@(t,state)ast_eoms(t,state,constants),tspan,initial_state,constants.ode_options);

fprintf('FINISHED SIM\n')
%% PLOT THE MOTION
plot_motion