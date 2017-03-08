% compute and plot energy for the inertial equations of motion

function [] = energy_plot_inertial(t,state_inertial,constants)


fontsize = constants.fontsize;
fontname = constants.fontname;

pos_cm = state_inertial(:,1:3);
vel_cm = state_inertial(:,4:6);
w_int2sc = state_inertial(:,16:18);

% extract and store the rotation matrix
Ra = zeros(3,3,length(t));
R_sc2int = zeros(3,3,length(t));

for ii = 1:length(t)
    Ra(:,:,ii) = ROT3(-constants.omega*t(ii)); % asteroid body frame to inertial frame
    R_sc2int(:,:,ii) = reshape(state_inertial(ii,7:15),3,3); % sc to inertial frame
end

% compute the total energy of the system
E = zeros(length(t),1);
T = zeros(length(t),1);
V = zeros(length(t),1);
J = zeros(length(t),1);

for ii = 1:length(t)
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
figure
subplot(1,2,1)
grid on
hold on
title('Change in E','interpreter','latex','fontsize',fontsize,'fontname',fontname)
xlabel('Time (sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname)
ylabel('$\frac{\Delta E}{E_0}$','interpreter','latex','fontsize',fontsize,'fontname',fontname)

E_diff = abs(E - E(1)); 
plot(t,abs(E_diff./E(1)))

subplot(1,2,2)
grid on
hold on
title('Change in E','interpreter','latex','fontsize',fontsize,'fontname',fontname)
xlabel('Time (sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname)
ylabel('$\Delta E$','interpreter','latex','fontsize',fontsize,'fontname',fontname)
plot(t,E_diff)


