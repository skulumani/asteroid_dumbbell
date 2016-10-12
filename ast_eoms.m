% EOMS for a dumbbell spacecraft model about an asteroid.
% The position/velocity vectors are defined in an asteroid body fixed
% rotating reference frame

function [state_dot] = ast_eoms(t,state,constants)

% unpack the state
pos = state(1:3); % location of the center of mass
vel = state(4:6);
R = reshape(state(7:15),3,3); % asteroid frame to sc body frame
W = state(16:18);

% unpack constants
m1 = constants.m1;
m2 = constants.m2;
lcg = constants.lcg;
l = constants.l;
J = constants.J;

% compute the potential/force on each mass
pos_m1 = pos - R'*lcg*[1;0;0];
pos_m2 = pos + R'*(l-lcg)*[1;0;0];

switch constants.pot_model
    case 'polyhedron'
        switch constants.asteroid_grav.num_f
            case 1024
                [U_m1,U_grad_m1,~, ~] = polyhedron_potential_mex_1024(pos_m1, constants.asteroid_grav);
                [U_m2,U_grad_m2,~, ~] = polyhedron_potential_mex_1024(pos_m2, constants.asteroid_grav);
            case 4092
                [U_m1,U_grad_m1,~, ~] = polyhedron_potential_mex_4092(pos_m1, constants.asteroid_grav);
                [U_m2,U_grad_m1,~, ~] = polyhedron_potential_mex_4092(pos_m2, constants.asteroid_grav);
        end
    case 'mascon'
        [U_m1,U_grad_m1] = mascon_potential(state,constants.asteroid_grav,constants);
        [U_m2,U_grad_m2] = mascon_potential(state,constants.asteroid_grav,constants);
    case 'matlab'
        [U_m1,U_grad_m1, ~, ~] = polyhedron_potential(pos, constants.asteroid_grav);
        [U_m2,U_grad_m2, ~, ~] = polyhedron_potential(pos, constants.asteroid_grav);
end

% compute the moments due to each mass
M1 = hat_map(pos_m1)*m1*U_grad_m1;
M2 = hat_map(pos_m2)*m2*U_grad_m2;

% state derivatives
pos_dot = vel;
vel_dot = 1/(m1+m2) * (m1*U_grad_m1 + m2*U_grad_m2) + [2*constants.omega*vel(2);-2*constants.omega*vel(1);0] + constants.omega^2*[pos(1);pos(2);0];
R_dot = reshape(R*hat_map(W),9,1);
W_dot = J\(M1 + M2 - hat_map(W)*J*W);

state_dot = [pos_dot;vel_dot;R_dot;W_dot];
end