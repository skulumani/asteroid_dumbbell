% 14 November 2016
% plot the energy variation of the inertial equations of motion as a
% function of the error tolerance of the ODE

% load constants
clc
clearvars
close all

% start_time = tic;

%% ASTEROID CONSTANTS
fprintf('LOADING CONSTANTS\n')
addpath(genpath(['.',filesep]));

constants = load_constants('castalia','low'); % only 1024 faces
asteroid_grav = polyhedron_shape_input(constants);
constants.asteroid_grav = asteroid_grav;

constants.pot_model = 'polyhedron'; % or mascon or matlab


%% SPACECRAFT CONSTANTS

constants.m1 = 100; % kg first mass
constants.m2 = 100; % kg second mass
constants.l = 0.002; % m rigid link
constants.lcg = constants.m2/(constants.m1+constants.m2)*constants.l;
constants.It = constants.m1*constants.lcg^2+constants.m2*(constants.l-constants.lcg)^2;
constants.Ia = 2/5*constants.m1*0.001^2 + 2/5*constants.m2*0.001^2;
constants.J = diag([constants.Ia,constants.It,constants.It]);

% initial condition
t_step = 0.1;
t_final = 100000;
constants.num_steps = t_final/t_step;

tspan = linspace(0,t_final,constants.num_steps);
initial_pos = [1.495746722510590;0.000001002669660;0.006129720493607]; % km for center of mass in body frame
initial_vel = [0.000000302161724;-0.000899607989820;-0.000000013286327]; % km/sec for COM in asteroid fixed frame
% convert from asteroid fixed frame to inertial frame
initial_vel = initial_vel + hat_map(constants.omega*[0;0;1])*initial_pos;
% initial_pos = [3;0;0];
% initial_vel = [0;0;0];
initial_R = reshape(eye(3,3),9,1); % transforms from dumbbell body frame to the inertial frame
initial_w = [0;0;0]; % angular velocity of dumbbell wrt to asteroid represented in sc body frame

initial_state = [initial_pos;initial_vel;initial_R;initial_w];


% ODE tolerances
ode_tol = logspace(-1,-10,10);

t_array = zeros(length(tspan),length(ode_tol));
state_array = zeros(length(tspan),length(initial_state),length(ode_tol));

for ii = 1:length(ode_tol)
    
    fprintf('ODE TOL: %3.2e\n',ode_tol(ii));
    ode_options = odeset('RelTol',ode_tol(ii),'AbsTol',ode_tol(ii));
    % simulate the ODE
    [t,state] = ode113(@(t,state)ast_eoms_inertial(t,state,constants),tspan,initial_state,ode_options);

    % save the state and energy behavior
    t_array(:,ii) = t;
    state_array(:,:,ii) = state;
    
    fprintf('  %4.2f%s done\n',ii/length(ode_tol)*100,'%');
end

fprintf('DONE\n')

%% save the array to mat function
save('./data/inertial_energy_behavior.mat','t_array','state_array','ode_tol','initial_state','constants','tspan')

%% plot all of the results
fontsize = constants.fontsize;
fontname = constants.fontname;
    
energy_fig = figure();
subplot(1,2,1)
grid on
hold all
title('Change in E','interpreter','latex','fontsize',fontsize,'fontname',fontname)
xlabel('Time (sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname)
ylabel('$\frac{\Delta E}{E_0}$','interpreter','latex','fontsize',fontsize,'fontname',fontname)

subplot(1,2,2)
grid on
hold all
title('Change in E','interpreter','latex','fontsize',fontsize,'fontname',fontname)
xlabel('Time (sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname)
ylabel('$\Delta E$','interpreter','latex','fontsize',fontsize,'fontname',fontname)
    
for jj=1:length(ode_tol)

    
    state_inertial = state_array(:,:,jj);
    t_inertial = t_array(:,jj);
    
    pos_cm = state_inertial(:,1:3);
    vel_cm = state_inertial(:,4:6);
    w_int2sc = state_inertial(:,16:18);
    
    % extract and store the rotation matrix
    Ra = zeros(3,3,length(t_inertial));
    R_sc2int = zeros(3,3,length(t_inertial));
    
    for ii = 1:length(t_inertial)
        Ra(:,:,ii) = ROT3(-constants.omega*t_inertial(ii)); % asteroid body frame to inertial frame
        R_sc2int(:,:,ii) = reshape(state_inertial(ii,7:15),3,3); % sc to inertial frame
    end
    
    % compute the total energy of the system
    E = zeros(length(t_inertial),1);
    T = zeros(length(t_inertial),1);
    V = zeros(length(t_inertial),1);
    J = zeros(length(t_inertial),1);
    
    for ii = 1:length(t_inertial)
        % compute the position and velocity of COM
        
        % the position of each mass in the asteroid body frame
        rho_1 = constants.lcg*[1;0;0];
        rho_2= (constants.l-constants.lcg)*[-1;0;0];
        
        % position of each mass in the asteroid frame
        z1 = Ra(:,:,ii)'*(pos_cm(ii,:)' + R_sc2int(:,:,ii) * rho_1);
        z2 = Ra(:,:,ii)'*(pos_cm(ii,:)' + R_sc2int(:,:,ii) * rho_2);
        
        z = Ra(:,:,ii)' * pos_cm(ii,:)'; % position of COM in asteroid frame
        
        % potential energy
        switch constants.pot_model
            case 'polyhedron'
                switch constants.asteroid_grav.num_f
                    case 1024
                        [U_m1,~,~, ~] = polyhedron_potential_mex_1024(z1, constants.asteroid_grav);
                        [U_m2,~,~, ~] = polyhedron_potential_mex_1024(z2, constants.asteroid_grav);
                        [U_com,~,~,~] = polyhedron_potential_mex_1024(z,constants.asteroid_grav);
                    case 4092
                        [U_m1,~,~, ~] = polyhedron_potential_mex_4092(z1, constants.asteroid_grav);
                        [U_m2,~,~, ~] = polyhedron_potential_mex_4092(z2, constants.asteroid_grav);
                        [U_com,~,~,~] = polyhedron_potential_mex_4092(z,constants.asteroid_grav);
                    case 32
                        [U_m1,~,~, ~] = polyhedron_potential_mex_32(z1, constants.asteroid_grav);
                        [U_m2,~,~, ~] = polyhedron_potential_mex_32(z2, constants.asteroid_grav);
                        [U_com,~,~,~] = polyhedron_potential_mex_32(z,constants.asteroid_grav);
                end
            case 'mascon'
                [U_m1,~] = mascon_potential(z1,constants.asteroid_grav,constants);
                [U_m2,~] = mascon_potential(z2,constants.asteroid_grav,constants);
                [U_com,~] = mascon_potential(z,constants.asteroid_grav,constants);
            case 'matlab'
                [U_m1,~, ~, ~] = polyhedron_potential(z1, constants.asteroid_grav);
                [U_m2,~, ~, ~] = polyhedron_potential(z2, constants.asteroid_grav);
                [U_com,~, ~, ~] = polyhedron_potential(z, constants.asteroid_grav);
        end
        
        
        T(ii) = 1/2*w_int2sc(ii,:)*constants.J*w_int2sc(ii,:)' + 1/2*(constants.m1+constants.m2)*vel_cm(ii,:)*vel_cm(ii,:)';
        V(ii) = -constants.m1*U_m1 - constants.m2*U_m2;
        E(ii) = T(ii) + V(ii);
        
    end
    
    %% DO THE PLOTTING
    
    % total energy change
    
    subplot(1,2,1)
    E_diff = abs(E - E(1));
    plot(t_inertial,abs(E_diff./E(1)))
    
    subplot(1,2,2)
    plot(t_inertial,E_diff)
    drawnow
end