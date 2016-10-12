% test function to call polyhedron_potential for code generation

% define inputs
constants = load_constants('castalia','true'); % false = full number of faces
asteroid_grav = polyhedron_shape_input(constants);

state = [1;0.2;0];


[U,Ug,Ug_mat, Ulap] = polyhedron_potential(state, asteroid_grav)
[U,Ug,Ug_mat, Ulap] = polyhedron_potential_mex_1024(state, asteroid_grav)


