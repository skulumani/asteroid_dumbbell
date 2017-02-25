%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Purpose: Rotation matrix about thrid axis (assumes row format)
%   b = a*dcm_a2b
%
%   Inputs: 
%       - gamma - rotation angle (rad)
%
%   Outpus: 
%       - rot3 - rotation matrix (3x3)
%
%   Dependencies: 
%       - none
%
%   Author: Shankar Kulumani 18 Aug 2012
%               - 15 Sept 2012 fixed error
%               - 26 Jan 2013 - converted to row format representation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rot3 = ROT3(gamma)

cos_gamma = cos(gamma);
sin_gamma = sin(gamma);

rot3 = [ cos_gamma -sin_gamma  0 ;   ...
        sin_gamma cos_gamma  0 ;   ...
            0         0       1 ];
        
end