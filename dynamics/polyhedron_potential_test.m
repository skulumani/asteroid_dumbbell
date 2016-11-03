% test function to call polyhedron_potential for code generation

% add functions to path
restoredefaultpath
addpath(genpath('./attitude_ref'));

% define inputs
constants = load_constants('castalia','true'); % false = full number of faces
asteroid_grav = polyhedron_shape_input(constants);

state = [1;0.2;0];


[U_mat,Ug_mat,Ug_mat_mat, Ulap_mat] = polyhedron_potential(state, asteroid_grav)
[U_mex,Ug_mex,Ug_mat_mex, Ulap_mex] = polyhedron_potential_mex_1024(state, asteroid_grav)


fprintf('Difference between mex and matlab functions\n')

fprintf('U_mat-U_mex: %5.4f\n',abs(U_mat-U_mex))
fprintf('Ug_mat-Ug_mex: %5.4f\n',abs(Ug_mat-Ug_mex))
fprintf('Ug_mat_mat-Ug_mat_mex: %5.4f\n',abs(Ug_mat_mat-Ug_mat_mex))
fprintf('Ulat_mat-Ulap_mex: %5.4f\n',abs(Ulap_mat-Ulap_mex))
