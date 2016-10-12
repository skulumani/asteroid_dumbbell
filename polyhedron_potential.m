% 19 Feb 2016
% polyhedral potential
function [U,U_grad, U_grad_mat, Ulaplace] = polyhedron_potential(state, asteroid_grav)
% all distances are in kilometers
% state = [1;0;0]; % kilometers

F = asteroid_grav.F;
V = asteroid_grav.V;

e1 = asteroid_grav.e1;
e2 = asteroid_grav.e2;
e3 = asteroid_grav.e3;

e1_face_map = asteroid_grav.e1_face_map;
e2_face_map = asteroid_grav.e2_face_map;
e3_face_map = asteroid_grav.e3_face_map;

e1_lock = zeros(size(e1_face_map));
e2_lock = zeros(size(e2_face_map));
e3_lock = zeros(size(e3_face_map));

e1_vertex_map = asteroid_grav.e1_vertex_map;
e2_vertex_map = asteroid_grav.e2_vertex_map;
e3_vertex_map = asteroid_grav.e3_vertex_map;

normal_face = asteroid_grav.normal_face;

e1_normal = asteroid_grav.e1_normal;
e2_normal = asteroid_grav.e2_normal;
e3_normal = asteroid_grav.e3_normal;

E1_edge = asteroid_grav.E1_edge;
E2_edge = asteroid_grav.E2_edge;
E3_edge = asteroid_grav.E3_edge;

F_face = asteroid_grav.F_face;

num_f = asteroid_grav.num_f;
num_e = asteroid_grav.num_e;
num_v = asteroid_grav.num_v;
G = asteroid_grav.G;
sigma = asteroid_grav.sigma;

% distance from state to each vertex 
r_v = V - repmat(state',num_v,1);

% vectorize w_face calculation
ri = r_v(F(:,1),:);
ri_norm=sqrt(ri(:,1).^2+ri(:,2).^2+ri(:,3).^2); 

rj = r_v(F(:,2),:);
rj_norm=sqrt(rj(:,1).^2+rj(:,2).^2+rj(:,3).^2); 

rk = r_v(F(:,3),:);
rk_norm=sqrt(rk(:,1).^2+rk(:,2).^2+rk(:,3).^2); 

rjrk_cross = cross(rj,rk);
num = sum(ri.*rjrk_cross,2);

rjrk_dot = sum(rj.*rk,2);
rkri_dot = sum(rk.*ri,2);
rirj_dot = sum(ri.*rj,2);

den = ri_norm.*rj_norm.*rk_norm + ri_norm.*rjrk_dot + rj_norm.*rkri_dot + rk_norm.*rirj_dot;

w_face = 2.*atan2(num,den);

% check if point is inside or outside the body
inside_check = sum(w_face); % zero when outside body and -G*sigma*4 pi on the inside

if inside_check < 1e-9 % outside the body
    
    r1i = r_v(e1_vertex_map(:,1),:);
    r1j = r_v(e1_vertex_map(:,2),:);
    r1i_norm = sqrt(r1i(:,1).^2+r1i(:,2).^2+r1i(:,3).^2);
    r1j_norm = sqrt(r1j(:,1).^2+r1j(:,2).^2+r1j(:,3).^2);
    e1_norm = sqrt(e1(:,1).^2+e1(:,2).^2+e1(:,3).^2);
    L1_edge = log((r1i_norm+r1j_norm+e1_norm)./(r1i_norm+r1j_norm-e1_norm));
    
    r2i = r_v(e2_vertex_map(:,1),:);
    r2j = r_v(e2_vertex_map(:,2),:);
    r2i_norm = sqrt(r2i(:,1).^2+r2i(:,2).^2+r2i(:,3).^2);
    r2j_norm = sqrt(r2j(:,1).^2+r2j(:,2).^2+r2j(:,3).^2);
    e2_norm = sqrt(e2(:,1).^2+e2(:,2).^2+e2(:,3).^2);
    L2_edge = log((r2i_norm+r2j_norm+e2_norm)./(r2i_norm+r2j_norm-e2_norm));
    
    r3i = r_v(e3_vertex_map(:,1),:);
    r3j = r_v(e3_vertex_map(:,2),:);
    r3i_norm = sqrt(r3i(:,1).^2+r3i(:,2).^2+r3i(:,3).^2);
    r3j_norm = sqrt(r3j(:,1).^2+r3j(:,2).^2+r3j(:,3).^2);
    e3_norm = sqrt(e3(:,1).^2+e3(:,2).^2+e3(:,3).^2);
    L3_edge = log((r3i_norm+r3j_norm+e3_norm)./(r3i_norm+r3j_norm-e3_norm));

    % calculate the potential at input state
    U_edge = 0;
    U_face = 0;
    
    U_grad_edge = zeros(3,1);
    U_grad_face = zeros(3,1);
    
    U_grad_mat_edge = zeros(3,3);
    U_grad_mat_face = zeros(3,3);
        
    % sum over edges
    for ii = 1:num_f
        
        % face contribution
        
        U_face = U_face + r_v(F(ii,1),:)*F_face(:,:,ii)*r_v(F(ii,1),:)'*w_face(ii,1);
        U_grad_face = U_grad_face + F_face(:,:,ii)*r_v(F(ii,1),:)'*w_face(ii,1);
        U_grad_mat_face = U_grad_mat_face + F_face(:,:,ii)*w_face(ii,1);
      
        % compute contributions for the three edges on this face but ignore if
        % it's a duplicate
        
        % edge 1
        if sum(e1_lock(ii,:)) == 0 % new edge
            
            U1 = r_v(e1_vertex_map(ii,1),:)*E1_edge(:,:,ii)*r_v(e1_vertex_map(ii,1),:)'*L1_edge(ii,1);
            U1_grad = E1_edge(:,:,ii)*r_v(e1_vertex_map(ii,1),:)'*L1_edge(ii,1);
            U1_grad_mat = E1_edge(:,:,ii)*L1_edge(ii,1);
            
            col = find(e1_face_map(ii,2:end) ~= 0);
            row = e1_face_map(ii,col+1);
            
            e1_lock(ii,1) = ii;
            e1_lock(ii,col+1) = row;
            % update lock
            if col(1) == 1 % adjacent face is also edge 1
                e1_lock(row,2) = ii;
            elseif col(1) == 2 % adjacent face is edge 2
                e2_lock(row,2) = ii;
            elseif col(1) == 3
                e3_lock(row,2) = ii;
            end
            
        else
            e1_lock(ii,1) = ii;
            
            U1 = 0;
            U1_grad = zeros(3,1);
            U1_grad_mat = zeros(3,3);
        end
        % edge 2
        if sum(e2_lock(ii,:)) == 0
            U2 = r_v(e2_vertex_map(ii,1),:)*E2_edge(:,:,ii)*r_v(e2_vertex_map(ii,1),:)'*L2_edge(ii,1);
            U2_grad = E2_edge(:,:,ii)*r_v(e2_vertex_map(ii,1),:)'*L2_edge(ii,1);
            U2_grad_mat = E2_edge(:,:,ii)*L2_edge(ii,1);
            
            col = find(e2_face_map(ii,2:end) ~= 0 );
            row = e2_face_map(ii,col+1);
            
            e2_lock(ii,1) = ii;
            e2_lock(ii,col+1) = row;
            
            % update lock
            if col(1) == 1 % duplicate edge is edge 1 on another face
                e1_lock(row,3) = ii;
            elseif col(1) == 2 % adjacent face is edge 2
                e2_lock(row,3) = ii;
            elseif col(1) == 3
                e3_lock(row,3) = ii;
            end
        else
            e2_lock(ii,1) = ii;
            
            U2 = 0;
            U2_grad = zeros(3,1);
            U2_grad_mat = zeros(3,3);
        end
        % edge 3
        if sum(e3_lock(ii,:)) == 0
            U3 = r_v(e3_vertex_map(ii,1),:)*E3_edge(:,:,ii)*r_v(e3_vertex_map(ii,1),:)'*L3_edge(ii,1);
            U3_grad = E3_edge(:,:,ii)*r_v(e3_vertex_map(ii,1),:)'*L3_edge(ii,1);
            U3_grad_mat = E3_edge(:,:,ii)*L3_edge(ii,1);
            
            col = find(e3_face_map(ii,2:end) ~= 0);
            row = e3_face_map(ii,col+1);
            
            e3_lock(ii,1) = ii;
            e3_lock(ii,col+1) = row;
            % update lock
            if col(1) == 1 % duplicate is in e1
                e1_lock(row,4) = ii;
            elseif col(1) == 2
                e2_lock(row,4) = ii;
            elseif col(1) == 3
                e3_lock(row,4) = ii;
            end
        else
            e3_lock(ii,1) = ii;
            
            U3 = 0;
            U3_grad = zeros(3,1);
            U3_grad_mat = zeros(3,3);
        end
        
        U_edge = U_edge + U1 + U2 + U3;
        U_grad_edge = U_grad_edge + U1_grad + U2_grad + U3_grad;
        U_grad_mat_edge = U_grad_mat_edge + U1_grad_mat + U2_grad_mat + U3_grad_mat;
        
    end
    % combine edge and face summations
    U = 1/2*G*sigma*U_edge-1/2*G*sigma*U_face;
    U_grad = -G*sigma*U_grad_edge + G*sigma*U_grad_face;
    U_grad_mat = G*sigma*U_grad_mat_edge -G*sigma*U_grad_mat_face;
    Ulaplace = -G*sigma*sum(w_face);
else
    U = 0;
    U_grad = zeros(3,1);
    U_grad_mat = zeros(3,3);
    Ulaplace = 0;
%     fprintf('INSIDE BODY!\n')
end
















