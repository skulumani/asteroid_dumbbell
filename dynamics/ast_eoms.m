function [state_dot] = ast_eoms(t,state,constants)

% unpack the state
pos = state(1:3);
vel = state(4:6);
% unpack constants

switch constants.pot_model
    case 'polyhedron'
        
        switch constants.asteroid_grav.num_f
            case 1024
                [U,U_grad,U_grad_mat, Ulaplace] = polyhedron_potential_mex_1024(pos, constants.asteroid_grav);
            case 4092
                [U,U_grad,U_grad_mat, Ulaplace] = polyhedron_potential_mex_4092(pos, constants.asteroid_grav);
        end
    case 'mascon'
        [U,U_grad] = mascon_potential(state,constants.asteroid_grav,constants);
    case 'matlab'
        [U,U_grad, U_grad_mat, Ulaplace] = polyhedron_potential(pos, constants.asteroid_grav);
end

% state derivatives
pos_dot = vel;
vel_dot = U_grad + [2*constants.omega*vel(2);-2*constants.omega*vel(1);0] + constants.omega^2*[pos(1);pos(2);0];

state_dot = [pos_dot;vel_dot];
end