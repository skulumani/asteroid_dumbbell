function [] = plot_inertial_motion(t,state_inertial,constants)

type = 'none';
animation_fname = 'dumbbell_inertial';

fontsize = constants.fontsize;
fontname = constants.fontname;

pos_cm = state_inertial(:,1:3);
vel_cm = state_inertial(:,4:6);
w_sc = state_inertial(:,16:18); % angular velocity of sc wrt inertial frame represented in the sc body frame
% extract and store the rotation matrix
R_sc2inertial = zeros(3,3,length(t));
R_ast2inertial = zeros(3,3,length(t));

for ii = 1:length(t)
    R_sc2inertial(:,:,ii) = reshape(state_inertial(ii,7:15),3,3); % sc to inertial frame
    R_ast2inertial(:,:,ii) = ROT3(-constants.omega*t(ii));
end

%% attitude animation
ani_plot = figure('color','white','position',[100 100 1124 868]);
% create subplots
h_tran_inertial = subplot(2,2,1);
grid on;
axis on
axis equal
hold on
title('Motion in inertial frame','interpreter','latex','fontsize',fontsize,'fontname',fontname)
ast_inertial = patch('Faces',constants.F,'Vertices',constants.V);
set(ast_inertial,'FaceLighting','gouraud','AmbientStrength',0.3,...
    'DiffuseStrength',0.8,'SpecularStrength',0.9,'BackFaceLighting','unlit','Facealpha',1,'Facecolor',[0.8 0.8 0.8], 'EdgeAlpha',0.0);
lightangle(90,0)
axis([-4 4 -4 4 -4 4]);
view(3)

h_inertial_attitude = subplot(2,2,2);
grid on;
hold on
title('Inertial Attitude','interpreter','latex','fontsize',fontsize,'fontname',fontname)
axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]); axis equal
% draw intertial frame
line([0 1],[0 0],[0 0],'color','k','linewidth',3);
line([0 0],[0 1],[0 0],'color','k','linewidth',3);
line([0 0],[0 0],[0 1],'color','k','linewidth',3);
view(3)

h_tran_body = subplot(2,2,3);
grid on
hold on
title('Motion in asteroid body frame','interpreter','latex','fontsize',fontsize,'fontname',fontname)
ast_body = patch('Faces',constants.F,'Vertices',constants.V);
set(ast_body,'FaceLighting','gouraud','AmbientStrength',0.3,...
    'DiffuseStrength',0.8,'SpecularStrength',0.9,'BackFaceLighting','unlit','Facealpha',1,'Facecolor',[0.8 0.8 0.8], 'EdgeAlpha',0.0);
lightangle(90,0)
axis([-4 4 -4 4 -4 4]);
view(3)

h_body_attitude = subplot(2,2,4);
grid on;
hold on
title('Asteroid Body Frame Attitude','interpreter','latex','fontsize',fontsize,'fontname',fontname)
axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]); axis equal
% draw asteroid body frame
line([0 1],[0 0],[0 0],'color','k','linewidth',3);
line([0 0],[0 1],[0 0],'color','k','linewidth',3);
line([0 0],[0 0],[0 1],'color','k','linewidth',3);
view(3)

tstep = 10;
switch type
    case 'gif'
        
        f = getframe(ani_plot);
        [im,map] = rgb2ind(f.cdata,256,'nodither');
    case 'movie'
        nFrames = length(t)/tstep;
        vidObj = VideoWriter(strcat(animation_fname,'.avi'));
        vidObj.Quality = 100;
        vidObj.FrameRate = 60;
        open(vidObj);
    case 'images'
    
    otherwise
        
end

% draw the attitude
for ii = 1:tstep:length(t)
    R_ast2i = R_ast2inertial(:,:,ii);
    R_i2ast = R_ast2inertial(:,:,ii)';
    
    % location of cm in both inertial and body frame of asteroid
    pos_i = pos_cm(ii,:)'; 
    pos_b = R_i2ast'*pos_i;
    
    % first body axis (first column of R)
    sc1_i = (R_sc2inertial(1,:,ii))'; % first body axis of SC represented in inertial frame
    sc1_b = R_i2ast'*sc1_i; % first body axis of SC represented in asteroid fixed frame
    sc1_bt = sc1_b + pos_b;
    sc1_it = sc1_i + pos_i;
    
    % asteroid body frame representations in the inertial frame
    ast1 = R_i2ast(:,1);
    ast2 = R_i2ast(:,2);
    ast3 = R_i2ast(:,3);
    
    % position of dumbell in the inertial frame
    pos_db1_i = pos_i-sc1_i/2;
    pos_db2_i = pos_i+sc1_i/2;
    
    % position of dumbbell in the asteroid fixed frame
    pos_db1_b = pos_b-sc1_b/2;
    pos_db2_b = pos_b+sc1_b/2;
    % plot SC in inertial frame
    set(0,'CurrentFigure',ani_plot)
    subplot(2,2,1)
    nV = constants.V*R_i2ast';
    set(ast_inertial,'Vertices',nV);
    db_it = line([pos_i(1) sc1_it(1)],[pos_i(2) sc1_it(2)],[pos_i(3) sc1_it(3)],'color','r','linewidth',3);
    db_it_mass = plot3([pos_db1_i(1) pos_db2_i(1)],[pos_db1_i(2) pos_db2_i(2)],[pos_db1_i(3) pos_db2_i(3)],'MarkerSize',20,'Marker','.','LineWidth',1,'Color','b');
    
    % attitude of dumbbell wrt inertial frame
    set(0,'CurrentFigure',ani_plot)
    subplot(2,2,2)
    % draw the asteroid body frame
    ast_x = line([0 ast1(1)],[0 ast1(2)],[0 ast1(3)],'color','r','linewidth',3);
    ast_y = line([0 ast2(1)],[0 ast2(2)],[0 ast2(3)],'color','g','linewidth',3);
    ast_z = line([0 ast3(1)],[0 ast3(2)],[0 ast3(3)],'color','b','linewidth',3);
    
    db_i = line([0 sc1_i(1)],[0 sc1_i(2)],[0 sc1_i(3)],'color','r','linewidth',3);
    db_i_mass = plot3([-sc1_i(1)/2 sc1_i(1)/2],[-sc1_i(2)/2 sc1_i(2)/2],[-sc1_i(3)/2 sc1_i(3)/2],'MarkerSize',20,'Marker','.','LineWidth',1,'Color','b');

    % position of dumbbell in asteroid body frame
    set(0,'CurrentFigure',ani_plot)
    subplot(2,2,3)
    db_bt = line([pos_b(1) sc1_bt(1)],[pos_b(2) sc1_bt(2)],[pos_b(3) sc1_bt(3)],'color','r','linewidth',3);
    db_bt_mass = plot3([pos_db1_b(1) pos_db2_b(1)],[pos_db1_b(2) pos_db2_b(2)],[pos_db1_b(3) pos_db2_b(3)],'MarkerSize',20,'Marker','.','LineWidth',1,'Color','b');
    
    % attitude wrt to asteroid fixed frame
    set(0,'CurrentFigure',ani_plot)
    subplot(2,2,4)
    db_b = line([0 sc1_b(1)],[0 sc1_b(2)],[0 sc1_b(3)],'color','r','linewidth',3);
    db_b_mass = plot3([-sc1_b(1)/2 sc1_b(1)/2],[-sc1_b(2)/2 sc1_b(2)/2],[-sc1_b(3)/2 sc1_b(3)/2],'MarkerSize',20,'Marker','.','LineWidth',1,'Color','b');
    xlabel(sprintf('t = %5.2f s',t(ii)));

    drawnow
    
    switch type
        case 'gif'
            
            frame = getframe(ani_plot);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            outfile = strcat(animation_fname,'.gif');
            
            % On the first loop, create the file. In subsequent loops, append.
            if ii==1
                imwrite(imind,cm,outfile,'gif','DelayTime',0,'loopcount',inf);
            else
                imwrite(imind,cm,outfile,'gif','DelayTime',0,'writemode','append');
            end
        case 'movie'
            writeVideo(vidObj,getframe(ani_plot));
        case 'images'
            outfile =  sprintf('%s_%d.jpg',animation_fname,ii);
            frame = getframe(ani_plot);
            imwrite(frame2im(frame),outfile);
        otherwise
            
    end
    
    delete(db_b);delete(db_b_mass)
    delete(db_i);delete(db_i_mass)
    delete(ast_x);delete(ast_y);delete(ast_z)
    delete(db_bt);delete(db_bt_mass)
    delete(db_it);delete(db_it_mass)
end


% Output the movie as an avi file
switch type
    case 'gif'
        
        
    case 'movie'
        close(vidObj);
        
    otherwise
        
end

%% Plot of all of the components
comp_fig = figure();
subplot(1,2,1)
grid on
hold all
plot(t,pos_cm(:,1), t,pos_cm(:,2),t,pos_cm(:,3))
title('Position components','interpreter','latex','fontsize',fontsize,'fontname',fontname);
xlabel('Time (sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname);
ylabel('Position (km)','interpreter','latex','fontsize',fontsize,'fontname',fontname);

subplot(1,2,2)
grid on
hold all
plot(t,vel_cm(:,1), t,vel_cm(:,2),t,vel_cm(:,3))
title('Velocity components','interpreter','latex','fontsize',fontsize,'fontname',fontname);
xlabel('Time (sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname);
ylabel('Velocity (km/sec)','interpreter','latex','fontsize',fontsize,'fontname',fontname);

% position plot
pos_fig = figure(); hold on
axis([-2 2 -2 2 -2 2]);
axis equal
% plot the asteroid
vertex_plotter(constants.F,constants.V,pos_fig);
plot3(pos_cm(:,1),pos_cm(:,2),pos_cm(:,3))
view(3)