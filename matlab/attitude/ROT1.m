%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Purpose: Rotation matrix about first axis (assumes row format)
%   b = a*dcm
%
%   Inputs: 
%       - alpha - rotation angle (rad)
%
%   Outpus: 
%       - rot1 - rotation matrix (3x3)
%
%   Dependencies: 
%       - none
%
%   Author: Shankar Kulumani 18 Aug 2012
%           26 Jan 2013 - converted to row format representation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rot1 = ROT1(alpha)

cos_alpha = cos(alpha);
sin_alpha = sin(alpha);

rot1 = [1     0          0      ;    ... 
        0  cos_alpha  -sin_alpha ;    ...
        0 sin_alpha  cos_alpha ];
end