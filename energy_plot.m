% compute and plot energy

function [] = energy_plot(t,state_body,constants)


fontsize = constants.fontsize;
fontname = constants.fontname;

pos_cm = state_body(:,1:3);
vel_cm = state_body(:,4:6);
w_ast2sc = state_body(:,16:18);
% extract and store the rotation matrix
R_ast2sc = zeros(3,3,length(t));

for ii = 1:length(t)
    R_ast2sc(:,:,ii) = reshape(state_body(ii,7:15),3,3); % asteroid to sc frame
end

% compute the total energy of the system
E = zeros(length(t),1);
T = zeros(length(t),1);
V = zeros(length(t),1);
J = zeros(length(t),1);

for ii = 1:length(t)
    % compute the position and velocity of COM
    
    % compute position of each mass in asteroid rotating frame
    vel_inertial = vel_cm(ii,:)' + hat_map(constants.omega*[0;0;1])*pos_cm(ii,:)';
    
    pos_m1 = pos_cm(ii,:)' - R_ast2sc(:,:,ii)'*constants.lcg*[1;0;0];
    pos_m2 = pos_cm(ii,:)' + R_ast2sc(:,:,ii)'*(constants.l - constants.lcg)*[1;0;0];
  
    % angular velocity of body wrt inertial frame represented in the
    % asteroid body fixed frame
    w_inertial2sc = constants.omega*[0;0;1] + w_ast2sc(ii,:)';
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
    
    T(ii) = 1/2*w_inertial2sc'*constants.J*w_inertial2sc + 1/2*(constants.m1+constants.m2)*(vel_inertial'*vel_inertial);
    V(ii) = -constants.m1*U_m1 - constants.m2*U_m2;
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
