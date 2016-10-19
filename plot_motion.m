% load the data
clc
% clearvars
close all

type = 'none';
animation_fname = 'dumbbell';
fontsize = 18;
fontname = 'Times';

% load planar_orbit.mat

pos_cm = state_body(:,1:3);
vel_cm = state_body(:,4:6);
w_ast2sc = state_body(:,16:18);
% extract and store the rotation matrix
R_ast2sc = zeros(3,3,length(t));

for ii = 1:length(t)
    R_ast2sc(:,:,ii) = reshape(state_body(ii,7:15),3,3); % asteroid to sc frame
end

% position plot
figure; hold on
axis([-2 2 -2 2 -2 2]);
axis equal
% plot the asteroid
vertex_plotter(constants.F,constants.V,figure(1));
plot3(pos_cm(:,1),pos_cm(:,2),pos_cm(:,3))
view(3)

% attitude plot
ani_plot = figure('color','white','position',[100 100 1124 868]);
% create subplots
h_s2a = subplot(2,2,1);grid on;hold on
title('Asteroid Body Frame')
axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]);axis equal
% draw intertial frame
line([0 1],[0 0],[0 0],'color','k','linewidth',3);
line([0 0],[0 1],[0 0],'color','k','linewidth',3);
line([0 0],[0 0],[0 1],'color','k','linewidth',3);
view(3)

h_s2i = subplot(2,2,2);grid on;hold on
title('Inertial Frame')
axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]); axis equal
% draw intertial frame
line([0 1],[0 0],[0 0],'color','k','linewidth',3);
line([0 0],[0 1],[0 0],'color','k','linewidth',3);
line([0 0],[0 0],[0 1],'color','k','linewidth',3);
view(3)

h_tran_body = subplot(2,2,3);grid on;
axis on
axis equal
hold on
ast_body = patch('Faces',constants.F,'Vertices',constants.V);
set(ast_body,'FaceLighting','gouraud','AmbientStrength',0.3,...
    'DiffuseStrength',0.8,'SpecularStrength',0.9,'BackFaceLighting','unlit','Facealpha',1,'Facecolor',[0.8 0.8 0.8], 'EdgeAlpha',0.0);
lightangle(90,0)
axis([-2 2 -2 2 -2 2]);
view(3)

h_tran_inertial = subplot(2,2,4);grid on;
axis on
axis equal
hold on
ast_inertial = patch('Faces',constants.F,'Vertices',constants.V);
set(ast_inertial,'FaceLighting','gouraud','AmbientStrength',0.3,...
    'DiffuseStrength',0.8,'SpecularStrength',0.9,'BackFaceLighting','unlit','Facealpha',1,'Facecolor',[0.8 0.8 0.8], 'EdgeAlpha',0.0);
lightangle(90,0)
axis([-2 2 -2 2 -2 2]);
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
        vidObj.FrameRate = 30;
        open(vidObj);
    case 'images'
    
    otherwise
        
end

% draw the body axes rotation
for ii = 1:tstep:length(t)
    R_i2ast = ROT3(constants.omega*t(ii));
    pos_i = R_i2ast'*pos_cm(ii,:)';
    
    % first body axis (first column of R)
    sc1_b = R_ast2sc(:,1,ii);
    sc1_i = R_i2ast*R_ast2sc(:,1,ii);
    sc1_bt = sc1_b + pos_cm(ii,:)';
    sc1_it = sc1_i + pos_i;
    
    ast1 = R_i2ast(:,1);
    ast2 = R_i2ast(:,2);
    ast3 = R_i2ast(:,3);
    % SC wrt asteroid body frame
    subplot(2,2,1)
    db_b = line([0 sc1_b(1)],[0 sc1_b(2)],[0 sc1_b(3)],'color','b','linewidth',3);
    db_b_mass = plot3([-sc1_b(1)/2 sc1_b(1)/2],[-sc1_b(2)/2 sc1_b(2)/2],[-sc1_b(3)/2 sc1_b(3)/2],'MarkerSize',20,'Marker','.','LineWidth',1,'Color','b');
    
    % SC wrt inertial frame
    subplot(2,2,2)
    db_i = line([0 sc1_i(1)],[0 sc1_i(2)],[0 sc1_i(3)],'color','b','linewidth',3);
    db_i_mass = plot3([-sc1_i(1)/2 sc1_i(1)/2],[-sc1_i(2)/2 sc1_i(2)/2],[-sc1_i(3)/2 sc1_i(3)/2],'MarkerSize',20,'Marker','.','LineWidth',1,'Color','b');
    % draw the body frame
    ast_x = line([0 ast1(1)],[0 ast1(2)],[0 ast1(3)],'color','r','linewidth',3);
    ast_y = line([0 ast2(1)],[0 ast2(2)],[0 ast2(3)],'color','g','linewidth',3);
    ast_z = line([0 ast3(1)],[0 ast3(2)],[0 ast3(3)],'color','b','linewidth',3);
    
    % translation/rotation in asteroid body frame
    subplot(2,2,3)
    db_bt = line([pos_cm(ii,1) sc1_bt(1)],[pos_cm(ii,2) sc1_bt(2)],[pos_cm(ii,3) sc1_bt(3)],'color','b','linewidth',3);
    db_bt_mass = plot3([pos_cm(ii,1)-sc1_b(1)/2 pos_cm(ii,1)+sc1_b(1)/2],[pos_cm(ii,2)-sc1_b(2)/2 pos_cm(ii,2)+sc1_b(2)/2],[pos_cm(ii,3)-sc1_b(3)/2 pos_cm(ii,3)+sc1_b(3)/2],'MarkerSize',20,'Marker','.','LineWidth',1,'Color','b');
    
    % inertial frame
    subplot(2,2,4)
    nV = constants.V*R_i2ast';
    set(ast_inertial,'Vertices',nV);
    
    db_it = line([pos_i(1) sc1_it(1)],[pos_i(2) sc1_it(2)],[pos_i(3) sc1_it(3)],'color','b','linewidth',3);
    db_it_mass = plot3([pos_i(1)-sc1_i(1)/2 pos_i(1)+sc1_i(1)/2],[pos_i(2)-sc1_i(2)/2 pos_i(2)+sc1_i(2)/2],[pos_i(3)-sc1_i(3)/2 pos_i(3)+sc1_i(3)/2],'MarkerSize',20,'Marker','.','LineWidth',1,'Color','b');
    xlabel(sprintf('t = %5.2f s',t(ii)));
    % delete things that should be removed
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
