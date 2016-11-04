% EOMS for a dumbbell spacecraft model about an asteroid.
% 
% These are the EOMS defined in the body frame of the asteroid.
% the kinematics are relative to the asteroid

function [state_dot] = ast_eoms_body(t,state,constants)

% unpack the state
pos = state(1:3); % location of the COM of dumbbell in asteroid fixed frame
vel = state(4:6);% vel of com wrt to asteroid expressed in the asteroid fixed frame
R = reshape(state(7:15),3,3); % sc body frame to asteroid body frame
w = state(16:18); % angular velocity of sc wrt asteroid frame and expressed in asteroid fixed frame

Ra = ROT3(-constants.omega*t); % asteroid body frame to inertial frame

% unpack constants
m1 = constants.m1;
m2 = constants.m2;
lcg = constants.lcg;
l = constants.l;
J = constants.J;
Jr = R*J*R';
Omega = constants.omega*[0;0;1]; % angular velocity vector of asteroid

% the position of each mass in the asteroid body frame
rho_1 = lcg*[1;0;0];
rho_2= (l-lcg)*[-1;0;0];

% position of each mass in the asteroid frame
z1 = (pos + R * rho_1);
z2 = (pos + R * rho_2);

z = pos; % position of COM in asteroid frame

switch constants.pot_model
    case 'polyhedron'
        switch constants.asteroid_grav.num_f
            case 1024
                [~,U_grad_m1,~, ~] = polyhedron_potential_mex_1024(z1, constants.asteroid_grav);
                [~,U_grad_m2,~, ~] = polyhedron_potential_mex_1024(z2, constants.asteroid_grav);
                [~,U_grad_com,~, ~] = polyhedron_potential_mex_1024(z, constants.asteroid_grav);
            case 4092
                [~,U_grad_m1,~, ~] = polyhedron_potential_mex_4092(z1, constants.asteroid_grav);
                [~,U_grad_m2,~, ~] = polyhedron_potential_mex_4092(z2, constants.asteroid_grav);
                [~,U_grad_com,~, ~] = polyhedron_potential_mex_4092(z, constants.asteroid_grav);
            case 32
                [~,U_grad_m1,~, ~] = polyhedron_potential_mex_32(z1, constants.asteroid_grav);
                [~,U_grad_m2,~, ~] = polyhedron_potential_mex_32(z2, constants.asteroid_grav);
                [~,U_grad_com,~, ~] = polyhedron_potential_mex_32(z, constants.asteroid_grav);
        end
    case 'mascon'
        [~,U_grad_m1] = mascon_potential(z1,constants.asteroid_grav,constants);
        [~,U_grad_m2] = mascon_potential(z2,constants.asteroid_grav,constants);
        [~,U_grad_com] = mascon_potential(z,constants.asteroid_grav,constants);
    case 'matlab'
        [~,U_grad_m1, ~, ~] = polyhedron_potential(z1, constants.asteroid_grav);
        [~,U_grad_m2, ~, ~] = polyhedron_potential(z2, constants.asteroid_grav);
        [~,U_grad_com, ~, ~] = polyhedron_potential(z, constants.asteroid_grav);
end

% force due to each mass expressed in asteroid body frame
F1 = m1*U_grad_m1;
F2 = m2*U_grad_m2;
F_com = (m1+m2)*U_grad_com;

% compute the moments due to each mass
M1 = m1*(rho_1*U_grad_m1'*R' - R*U_grad_m1*rho_1');
M2 = m2*(rho_2*U_grad_m2'*R' - R*U_grad_m2*rho_2');

% apply the vee map to get a vector
M1 = vee_map(M1);
M2 = vee_map(M2);

% state derivatives
pos_dot = vel - hat_map(Omega)*pos;
vel_dot = 1/(m1+m2) * (F1 + F2 - (m1+m2) * hat_map(Omega)*vel);
% vel_dot = 1/(m1+m2) * (F_com) ;
R_dot = hat_map(w)*R - hat_map(Omega)*R;
R_dot = reshape(R_dot,9,1);
w_dot = Jr\ (M1 + M2 - hat_map(Omega)*(Jr*w) - (( hat_map(w) - hat_map(Omega))*Jr + Jr*( hat_map(Omega)-hat_map(w) ))*w);

state_dot = [pos_dot;vel_dot;R_dot;w_dot];
end















