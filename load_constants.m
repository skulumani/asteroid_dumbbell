% 18 Feb 2016
% load constants of asteroid

function constants = load_constants(asteroid,reduce_flag)
% define constants
constants.G = 6.673e-20; % km^3/kg/sec^2

constants.periodic_tspan = [0 30000]; % seconds
constants.RelTol = 1e-9;
constants.AbsTol = 1e-9;
constants.tol = 1e-6;
constants.diffcorr_plot = 0;
constants.jacobi_step = 1e-9;
% 433 Eros from book

switch asteroid
    case 'itokawa'
        % asteroid Itokawa
        constants.G = 6.673e-20; % km^3/kg/sec^2
        constants.M = 3.51e10; % kg
        constants.sigma = 1.9; % grams/cm^3
        constants.mu = constants.G*constants.M; % actual grav parameter of asteroid
        constants.axes = [535 294 209]; % m
        constants.Ixx = constants.M/5*(constants.axes(2)^2+constants.axes(3)^2); % kg m^2
        constants.Iyy = constants.M/5*(constants.axes(1)^2+constants.axes(3)^2);
        constants.Izz = constants.M/5*(constants.axes(1)^2+constants.axes(2)^2);
        constants.omega = 2*pi/12.132/3600; % rotation period rad/sec
        constants.mass_param = (constants.Iyy-constants.Ixx)/(constants.Izz-constants.Ixx);
        constants.res_radius = (constants.mu/constants.omega^2)^(1/3);
        constants.dist_scale = constants.res_radius;
        constants.time_scale = constants.omega;
        constants.C20 = -1/2*(constants.Izz-constants.Ixx)*(2-constants.mass_param)/constants.dist_scale^2/constants.M;
        constants.C22 = 1/4*(constants.Izz-constants.Ixx)*constants.mass_param/constants.dist_scale^2/constants.M;
        % logic to decide which asteroid to open
        % ITOKAWA
        % rearrange the matrices
        [filename, pathname] = uigetfile('itokawa shape data');
        shape_data = dlmread(strcat(pathname,filename));
        num_vertices = shape_data(1,1);
        num_facets = shape_data(1,2);
        
        % vertices in body fixed frame in km
        constants.V = shape_data(2:num_vertices+1,2:end);
        % each face defines the three vertices which define it
        constants.F = shape_data(num_vertices+2:end,2:end);
        
    case 'castalia'
        % add directory to path
        addpath(strcat('.',filesep,'CASTALIA'))
        % 4769 Casstalia
        constants.G = 6.673e-20; % km^3/kg/sec^2
        constants.M = 1.4091e12; % kg
        constants.sigma = 2.1; % grams/cm^3
        constants.mu = constants.G*constants.M; % km^3/s^2 actual grav parameter of asteroid
        constants.axes = [1.6130 0.9810 0.8260]*1e3 / 2; % semi axis length m
        constants.Ixx = constants.M/5*(constants.axes(2)^2+constants.axes(3)^2); % kg m^2
        constants.Iyy = constants.M/5*(constants.axes(1)^2+constants.axes(3)^2);
        constants.Izz = constants.M/5*(constants.axes(1)^2+constants.axes(2)^2);
        constants.omega = 2*pi/4.07/3600; % rotation period rad/sec
        constants.mass_param = (constants.Iyy-constants.Ixx)/(constants.Izz-constants.Ixx);
        constants.res_radius = (constants.mu/constants.omega^2)^(1/3);
        constants.dist_scale = constants.res_radius; % meters
        constants.time_scale = constants.omega;
        constants.C20 = -1/2*(constants.Izz-constants.Ixx)*(2-constants.mass_param)/constants.dist_scale^2/constants.M;
        constants.C22 = 1/4*(constants.Izz-constants.Ixx)*constants.mass_param/constants.dist_scale^2/constants.M;
        % constants.C20 = -7.275e-2;
        % constants.C22 = 2.984e-2;
        
        load castalia.mat

        switch reduce_flag
            case 'true' % reduce number of faces for mex function
                fprintf('Reducing number of faces\n')
                [F,V] = reducepatch(F,V,1024); % must use 1024 for polyhedron_potential_mex
            otherwise
                fprintf('No reduction\n')
               	 
        end
        
        fprintf('Polyhedron Model: %g faces, %g vertices\n',length(F),length(V));
        % rotate the body vectors into an orientation that matches
        % scheeres1996 - Orbits close to asteroid 4769 Castalia

        constants.V = (ROT3(pi)*V')';
        constants.F = F;
        
        % convert to lat, long and radius
        % long = atan2(y/x)
        % lat = asin(z/r)
        % r = norm(V)
        
        % calculate the distance
        constants.r = sqrt(constants.V(:,1).^2 + constants.V(:,2).^2 + constants.V(:,3).^2);
        constants.long = atan2(constants.V(:,2),constants.V(:,1)) * 180/pi;
        constants.lat = asin(constants.V(:,3)./constants.r) * 180/pi;
        
        % sort in order of increasing radius
        [constants.r, index] = sort(constants.r);
        constants.long = constants.long(index);
        constants.lat = constants.lat(index);
        
        
end
