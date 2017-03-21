% test relative equations of motion

%% ASTEROID CONSTANTS
fprintf('LOADING CONSTANTS\n')
addpath(genpath(['.',filesep]));

constants = load_constants('castalia','low'); % only 1024 faces
asteroid_grav = polyhedron_shape_input(constants);
constants.asteroid_grav = asteroid_grav;

constants.pot_model = 'polyhedron'; % or mascon or matlab
constants.ode_options = odeset('RelTol',1e-6,'AbsTol',1e-6);

%% SPACECRAFT CONSTANTS

constants.m1 = 100; % kg first mass
constants.m2 = 100; % kg second mass
constants.l = 0.002; % km rigid link
constants.r = 0.001; % km radius of each spherical mass 

%% INERTIAL SIMULATION
fprintf('INERTIAL SIMULATION!\n')

t_final = 1e5;
constants.num_steps = 1e5;

tspan = linspace(0,t_final,constants.num_steps);
initial_pos = [1.495746722510590;0.000001002669660;0.006129720493607]; % km for center of mass in body frame
initial_vel = [0.000000302161724;-0.000899607989820;-0.000000013286327]; % km/sec for COM in asteroid fixed frame

initial_state = [initial_pos;initial_vel];
[t, state] = ode113(@(t,state)ast_eoms(t,state,constants),tspan,initial_state,constants.ode_options);

fprintf('FINISHED INERTIAL SIM\n')

plot3(state(:,1), state(:,2), state(:,3)) 