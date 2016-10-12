% load the data
clc
clearvars
close all

load planar_orbit.mat

pos_cm = state_body(:,1:3);
vel_cm = state_body(:,4:6);
W_ast2sc = state_body(16:18);

% position plot
figure(1)
axis([-1 1 -1 1 -1 1])
axis equal
% plot the asteroid
vertex_plotter(constants.F,constants.V,figure(1));
plot3(pos_cm(:,1),pos_cm(:,2),pos_cm(:,3))

% attitude plot
figure
% create subplots
h_s2a = subplot(2,2,1);grid on;
h_s2i = subplot(2,2,2);grid on;
h_tran_body = subplot(2,2,3);grid on;
h_tran_inertial = subplot(2,2,4);grid on;

% extract and store the rotation matrix
R_ast2sc = zeros(3,3,length(t));

% draw the body axes rotation
for ii = 1:length(t)
    cla
    axis([-1 1 -1 1 -1 1])
    % draw intertial frame
    line([0 1],[0 0],[0 0],'color','k','linewidth',3);
    line([0 0],[0 1],[0 0],'color','k','linewidth',3);
    line([0 0],[0 0],[0 1],'color','k','linewidth',3);

    R_ast2sc(:,:,ii) = reshape(state_body(ii,7:15),3,3); % asteroid to sc frame
    
    % first body axis (first column of R)
    b1 = R_ast2sc(:,1,ii);
    
    line([0 b1(1)],[0 b1(2)],[0 b1(3)],'color','b','linewidth',3)
    
    % display the current simulation time in the title
    title(sprintf('Sim Time: %5.2f sec',t(ii)))
    
    drawnow
end