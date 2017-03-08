% compute and plot energy

function [] = energy_plot_body(t,state_body,constants)


fontsize = constants.fontsize;
fontname = constants.fontname;
m1 = constants.m1;
m2 = constants.m2;

pos_cm = state_body(:,1:3);% location of the COM of dumbbell in asteroid fixed frame
vel_cm = state_body(:,4:6);% vel of com wrt to asteroid expressed in the asteroid fixed frame
w_sc2ast = state_body(:,16:18);% angular velocity of sc wrt asteroid frame and expressed in asteroid fixed frame
% extract and store the rotation matrix
R_sc2ast = zeros(3,3,length(t));% sc body frame to asteroid body frame

for ii = 1:length(t)
    R_sc2ast(:,:,ii) = reshape(state_body(ii,7:15),3,3); % sc to asteroid body frame
end

rho_1 = constants.lcg*[1;0;0];
rho_2= (constants.l-constants.lcg)*[-1;0;0];

% compute the total energy of the system
E = zeros(length(t),1);
T = zeros(length(t),1);
V = zeros(length(t),1);
J = zeros(length(t),1);

for ii = 1:length(t)
    
    
    pos_m1 = pos_cm(ii,:)' + R_sc2ast(:,:,ii)*rho_1;
    pos_m2 = pos_cm(ii,:)' + R_sc2ast(:,:,ii)*rho_2;
  
    % potential energy
    switch constants.pot_model
        case 'polyhedron'
            switch constants.asteroid_grav.num_f
                case 1024
                    [U_m1,~,~, ~] = polyhedron_potential_mex_1024(pos_m1, constants.asteroid_grav);
                    [U_m2,~,~, ~] = polyhedron_potential_mex_1024(pos_m2, constants.asteroid_grav);
                    [U_com,~,~,~] = polyhedron_potential_mex_1024(pos_cm(ii,:)',constants.asteroid_grav);
                case 4092
                    [U_m1,~,~, ~] = polyhedron_potential_mex_4092(pos_m1, constants.asteroid_grav);
                    [U_m2,~,~, ~] = polyhedron_potential_mex_4092(pos_m2, constants.asteroid_grav);
                    [U_com,~,~,~] = polyhedron_potential_mex_4092(pos_cm(ii,:)',constants.asteroid_grav);
                case 32
                    [U_m1,~,~, ~] = polyhedron_potential_mex_32(pos_m1, constants.asteroid_grav);
                    [U_m2,~,~, ~] = polyhedron_potential_mex_32(pos_m2, constants.asteroid_grav);
                    [U_com,~,~,~] = polyhedron_potential_mex_32(pos_cm(ii,:)',constants.asteroid_grav);
            end
        case 'mascon'
            [U_m1,~] = mascon_potential(pos_m1,constants.asteroid_grav,constants);
            [U_m2,~] = mascon_potential(pos_m2,constants.asteroid_grav,constants);
            [U_com,~] = mascon_potential(pos_cm(ii,:)',constants.asteroid_grav,constants);
        case 'matlab'
            [U_m1,~, ~, ~] = polyhedron_potential(pos_m1, constants.asteroid_grav);
            [U_m2,~, ~, ~] = polyhedron_potential(pos_m2, constants.asteroid_grav);
            [U_com,~, ~, ~] = polyhedron_potential(pos_cm(ii,:)', constants.asteroid_grav);
    end
    
    T(ii) = 1/2*(m1+m2)*vel_cm(ii,:)*vel_cm(ii,:)' + 1/2*w_sc2ast(ii,:)*constants.J*w_sc2ast(ii,:)';
    V(ii) = -m1*U_m1 - m2*U_m2;
    E(ii) = T(ii) + V(ii);

    J(ii) = 1/2*(vel_cm(ii,:)*vel_cm(ii,:)') - 1/2*constants.omega^2*(pos_cm(ii,1)^2+pos_cm(ii,2)^2) - U_com;
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

% change in J
figure
subplot(1,2,1)
grid on
hold on
title('Change in J','interpreter','latex','fontsize',fontsize,'fontname',fontname)
xlabel('Time (sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname)
ylabel('$\frac{\Delta J}{J_0}$','interpreter','latex','fontsize',fontsize,'fontname',fontname)

J_diff = abs(J - J(1)); 
plot(t,abs(J_diff./J(1)))

subplot(1,2,2)
grid on
hold on
title('Change in J','interpreter','latex','fontsize',fontsize,'fontname',fontname)
xlabel('Time (sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname)
ylabel('$\Delta J$','interpreter','latex','fontsize',fontsize,'fontname',fontname)
plot(t,J_diff)
