% 8 June 15
% skew symmetric operator

function mat = hat_map(vec)
% maps a 3-vec to a skew symmetric matrix
mat = zeros(3,3);

mat(1,2) = -vec(3);
mat(1,3) = vec(2);
mat(2,1) = vec(3);
mat(2,3) = -vec(1);
mat(3,1) = -vec(2);
mat(3,2) = vec(1);

