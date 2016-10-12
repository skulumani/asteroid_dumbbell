%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Purpose: Rotation matrix about second axis (assumes row format)
%   b = a*dcm
%
%   Inputs: 
%       - beta - rotation angle (rad)
%
%   Outpus: 
%       - rot2 - rotation matrix (3x3)
%
%   Dependencies: 
%       - none
%
%   Author: Shankar Kulumani 18 Aug 2012
%           26 Jan 2013 - converted to row format representation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rot2 = ROT2(beta)


cos_beta = cos(beta);
sin_beta = sin(beta);

rot2 = [cos_beta  0  sin_beta;   ...
           0      1      0    ;   ...
        -sin_beta  0  cos_beta ];
end
