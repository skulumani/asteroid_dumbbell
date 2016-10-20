% EOMS for a dumbbell spacecraft model about an asteroid.
% The position/velocity vectors are defined in an asteroid body fixed
% rotating reference frame

function [state_dot] = ast_eoms(t,state,constants)

% unpack the state
pos = state(1:3); % location of the center of mass in asteroid body frame
vel = state(4:6);% vel of com in asteroid body frame
R = reshape(state(7:15),3,3); % sc body frame to asteroid body frame 
w = state(16:18); % asteroid body frame to sc body frame expressed in asteroid frame

% unpack constants
m1 = constants.m1;
m2 = constants.m2;
lcg = constants.lcg;
l = constants.l;
J = constants.J;

% the position of each mass in the asteroid body frame
pos_m1 = pos - R*lcg*[1;0;0];
pos_m2 = pos + R*(l-lcg)*[1;0;0];

switch constants.pot_model
    case 'polyhedron'
        switch constants.asteroid_grav.num_f
            case 1024
                [~,U_grad_m1,~, ~] = polyhedron_potential_mex_1024(pos_m1, constants.asteroid_grav);
                [~,U_grad_m2,~, ~] = polyhedron_potential_mex_1024(pos_m2, constants.asteroid_grav);
                [~,U_grad_com,~, ~] = polyhedron_potential_mex_1024(pos, constants.asteroid_grav);
            case 4092
                [~,U_grad_m1,~, ~] = polyhedron_potential_mex_4092(pos_m1, constants.asteroid_grav);
                [~,U_grad_m2,~, ~] = polyhedron_potential_mex_4092(pos_m2, constants.asteroid_grav);
                [~,U_grad_com,~, ~] = polyhedron_potential_mex_4092(pos, constants.asteroid_grav);
        end
    case 'mascon'
        [~,U_grad_m1] = mascon_potential(pos_m1,constants.asteroid_grav,constants);
        [~,U_grad_m2] = mascon_potential(pos_m2,constants.asteroid_grav,constants);
        [~,U_grad_com] = mascon_potential(pos,constants.asteroid_grav,constants);
    case 'matlab'
        [~,U_grad_m1, ~, ~] = polyhedron_potential(pos_m1, constants.asteroid_grav);
        [~,U_grad_m2, ~, ~] = polyhedron_potential(pos_m2, constants.asteroid_grav);
        [~,U_grad_com, ~, ~] = polyhedron_potential(pos, constants.asteroid_grav);
end

% force due to each mass expressed in asteroid body frame
F1 = m1*U_grad_m1;
F2 = m2*U_grad_m2;
F_com = (m1+m2)*U_grad_com;

% compute the moments due to each mass
M1 = hat_map(R*lcg*[1;0;0])*F1;
M2 = hat_map(R*(l-lcg)*[1;0;0])*F2;

% state derivatives
pos_dot = vel;
vel_dot = 1/(m1+m2) * (F1+F2) - 2*hat_map(constants.omega*[0;0;1])*vel - hat_map(constants.omega*[0;0;1])*hat_map(constants.omega*[0;0;1])*pos;
% vel_dot = 1/(m1+m2) * (F_com) - 2*hat_map(constants.omega*[0;0;1])*vel - hat_map(constants.omega*[0;0;1])*hat_map(constants.omega*[0;0;1])*pos;
R_dot = R*hat_map(R'*w);
R_dot = reshape(R_dot,9,1);
w_dot = J\(M1 + M2 - hat_map(constants.omega*[0;0;1]+ w)*(J*(constants.omega*[0;0;1] + w)));

state_dot = [pos_dot;vel_dot;R_dot;w_dot];
end