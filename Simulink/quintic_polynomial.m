function coefficients = quintic_polynomial(T_max,r0,rf,v0,vf,a0,af)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This function produces the coefficients for a reference trajectory that
%   meets conditions on initial position, velocity, and acceleration (r0,
%   v0, a0) and the final position, velocity, and acceleration (rf, vf, af)
% 
% Andrea L'Afflitto
% Virginia Tech
% 06/25/2020
% 
% Input:
% r0: scalar initial positions
% zf: scalar final positions
% T_max: final time
% 
% Output:
% coefficients: vector of polynomial coefficients so that the reference
% trajectory at time t is given by coefficients'*[1;t;t^2;t^3;t^4;t^5]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


M = [1, 0, 0, 0, 0, 0;
     1, T_max, T_max^2, T_max^3, T_max^4, T_max^5;
     0, 1, 0, 0, 0, 0;
     0, 1, 2*T_max, 3*T_max^2, 4*T_max^3, 5*T_max^4;
     0, 0, 2, 0, 0, 0;
     0, 0, 2, 6*T_max, 12*T_max^2, 20*T_max^3];
 
coefficients = M\[r0;rf;v0;vf;a0;af];