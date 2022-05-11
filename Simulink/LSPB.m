function [q_ref,q_ref_dot,q_ref_ddot] = LSPB(tf,q0,qf,t)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function produces the reference position, velocity, and
% acceleration at time t for a robot with n DOF by applying the linear
% segments with parabolic blends (LSPB) algorithm outlined in Spong's
% book on pp. 192-195
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input:
% q0: vector of initial positions of dimension n
% qf: vector of final positions of dimension n
% tf: final time
% t: time at which the reference trajectory, the reference velocity, and the
%    reference acceleration must be evaluated. Note t must be between 0
%    and tf

% Output:
% q_ref: robot's reference position at time t
% q_ref_dot: robot's reference velocity at time t
% q_ref_ddot: robot's reference acceleration at time t

V = 1.6.*(qf-q0)./tf; % Reference velocity between tb and tf-tb chosen to meet the constraints discussed in the textbook
                      % Note that V is a vector of dimension n.

tb = (q0-qf+V*tf)./V; % See equation (5.24) of Spong's book.
                      % Note that tb is a vector of dimension n.
                      
alpha = V./tb;

% Initializing reference trajectory
q_ref = zeros(length(q0),1);
q_ref_dot = zeros(length(q0),1);
q_ref_ddot = zeros(length(q0),1);

% Reference trajectory and its derivatives for each link depending on time in simulation
for jj =1:length(q0)
    if and(t>=0,t<=tb(jj))
        q_ref(jj) = q0(jj) + alpha(jj)./2.*t^2;
        q_ref_dot(jj) = alpha(jj).*t;
        q_ref_ddot(jj) = alpha(jj);
    elseif and(t>tb(jj),t<=(tf-tb(jj)))
        q_ref(jj) = (qf(jj)+q0(jj)-V(jj).*tf)/2+V(jj).*t;
        q_ref_dot(jj) = V(jj);
        q_ref_ddot(jj) = 0;
    elseif and(t>(tf-tb(jj)),t<=tf)
        q_ref(jj) = qf(jj)-(alpha(jj).*tf^2)./2+alpha(jj).*tf.*t-alpha(jj)./2.*t^2;
        q_ref_dot(jj) = alpha(jj).*tf-alpha(jj)*t;
        q_ref_ddot(jj) = -alpha(jj);
    end
end
    
    
    
    
        