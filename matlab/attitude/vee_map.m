% 11 June 15
% vee map function to take a skew symmetric matrix and map it to a 3 vector

function [vec] = vee_map(mat)

x1 = mat(3,2)-mat(2,3);
x2 = mat(1,3) - mat(3,1);
x3 = mat(2,1)-mat(1,2);

vec = 1/2*[x1;x2;x3];