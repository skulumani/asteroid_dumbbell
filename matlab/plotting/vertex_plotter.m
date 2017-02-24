% 2 Feb 2016
% plot shape model on figure

function vertex_plotter(F,V,fig_handle)

set(0,'CurrentFigure',fig_handle)
axis equal
hold on
% title('Asteroid','interpreter','latex')
ast = patch('Faces',F,'Vertices',V);

set(ast,'FaceLighting','gouraud','AmbientStrength',0.5,...
    'Facealpha',1,'Facecolor','green', 'EdgeAlpha',0.5);
% 
% % draw body axis
% axis_data = get(gca);
% xmin = axis_data.XLim(1);
% xmax = axis_data.XLim(2);
% ymin = axis_data.YLim(1);
% ymax = axis_data.YLim(2);
% zmin = axis_data.ZLim(1);
% zmax = axis_data.ZLim(2);

% % I, J ,K vectors
% plot3([xmin,xmax],[0 0],[0 0],'red','Linewidth',3); plot3(xmax,0,0,'r>','Linewidth',1.5);
% plot3([0 0],[ymin,ymax],[0 0],'green','Linewidth',3); plot3(0,ymax,0,'g>','Linewidth',1.5);
% plot3([0 0],[0 0],[zmin,zmax],'blue','Linewidth',3); plot3(0,0,zmax,'b^','Linewidth',1.5);

% index = 7;
% % draw vertices
% plot3([zeros(length(index)),V(Fa(index),1)],[zeros(length(index)),V(Fa(index),2)],[zeros(length(index)),V(Fa(index),3)],'r.','Markersize',20)
% plot3(V(Fb(index),1),V(Fb(index),2),V(Fb(index),3),'g.','Markersize',20)
% plot3(V(Fc(index),1),V(Fc(index),2),V(Fc(index),3),'b.','Markersize',20)
% 
% % draw edges
% plot3([V(Fa(index),1),e1(index,1)+V(Fa(index),1)],[V(Fa(index),2),e1(index,2)+V(Fa(index),2)],[V(Fa(index),3),e1(index,3)+V(Fa(index),3)],'red','Linewidth',3)
% plot3([V(Fb(index),1),e2(index,1)+V(Fb(index),1)],[V(Fb(index),2),e2(index,2)+V(Fb(index),2)],[V(Fb(index),3),e2(index,3)+V(Fb(index),3)],'green','Linewidth',3)
% plot3([V(Fc(index),1),e3(index,1)+V(Fc(index),1)],[V(Fc(index),2),e3(index,2)+V(Fc(index),2)],[V(Fc(index),3),e3(index,3)+V(Fc(index),3)],'blue','Linewidth',3)
% 
% % draw face normal
% plot3([V(Fa(index),1),normal_face(index,1)+V(Fa(index),1)],[V(Fa(index),2),normal_face(index,2)+V(Fa(index),2)],[V(Fa(index),3),normal_face(index,3)+V(Fa(index),3)],'black','Linewidth',3)
% 
% % draw three edge normals
% plot3([V(Fa(index),1),e1_normal(index,1)+V(Fa(index),1)],[V(Fa(index),2),e1_normal(index,2)+V(Fa(index),2)],[V(Fa(index),3),e1_normal(index,3)+V(Fa(index),3)],'red','Linewidth',3)
% plot3([V(Fb(index),1),e2_normal(index,1)+V(Fb(index),1)],[V(Fb(index),2),e2_normal(index,2)+V(Fb(index),2)],[V(Fb(index),3),e2_normal(index,3)+V(Fb(index),3)],'green','Linewidth',3)
% plot3([V(Fc(index),1),e3_normal(index,1)+V(Fc(index),1)],[V(Fc(index),2),e3_normal(index,2)+V(Fc(index),2)],[V(Fc(index),3),e3_normal(index,3)+V(Fc(index),3)],'blue','Linewidth',3)

