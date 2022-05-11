function u = control_law_perfect_cancellation(q,q_dot,M,C,Kq,e,e_dot,e_int,q_ref_ddot,KP,KD,KI)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function uses inverse dynamics to determine the torque required 
% to actuate the joints of the WidowX 200 robotic arm to follow a 
% specified reference trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Control law
u = C*q_dot + Kq + M*(- KP*e - KD*e_dot - KI*e_int + q_ref_ddot);
