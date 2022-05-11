function [qf] = inverse_kinematics(alpha,z_5,p,d1,a2,a3,d5,bounds,bounds_dist_from_center,point_dist_from_center)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function uses Denavit-Hartenberg inverse kinematis to calculate the 
% angles required to reach a desired end effector position, axis, and 
% gripper angle for the WidowX 200 Robotic Arm.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gripper angle is alpha
theta_5 = alpha;

% Base angle calculated from the desired x and y position
theta_1 = -atan2(p(2),p(1));

% Statement ensures the arm turns a maximum of pi radians
if theta_1<-pi/2
    theta_1 = theta_1+2*pi;
end

%Calculating intermediate parameters
z5X = double(sqrt(z_5(1,1)^2 + z_5(2,1)^2));
phi = atan2(z_5(3,1),z5X);
p_X = double(sqrt(p(1)^2 + p(2)^2));
rho_X = p_X - d5*cos(phi);
rho_z = p(3) - d1-d5*sin(phi);
C = double((rho_X^2 + rho_z^2 + a2^2 - a3^2)/(2*a2));

% If the angle contains imaginary parts, this will terminate the function
% and trigger an error message
quit = ~isreal(double(sqrt(rho_z^2 + rho_X^2 - C^2)));
if quit
    close
    
    set(figure,'Color','white')
    plot(bounds_dist_from_center(1:end),bounds(1:end,3),'-r','LineWidth',2)
    hold on
    plot(point_dist_from_center,p(3),'r*','MarkerSize',8)
    title('End Effector Range')
    l= legend('end effector limits', 'desired position');
    set(l,'interpreter','latex','fontsize',10);
    set(gca,'fontsize',15)
    xlabel('distance from center axis ($\sqrt{x^2+y^2}$)','interpreter','latex','fontsize',20)
    ylabel('height (z)','interpreter','latex','fontsize',20)
    axis tight
    
    error('Desired position is inside the range of the robotic arm but cannot be reached. Try a different end effector axis orientation.');
end

% Calculating angles 2, 3, and 4
theta_2 = 2*atan2((rho_z + double(sqrt(rho_z^2 + rho_X^2 - C^2))),(rho_X + C));
theta_3 = atan2((rho_z - a2*sin(theta_2)),(rho_X - a2*cos(theta_2))) - theta_2;
theta_4 = phi - theta_3 - theta_2;

% Creating vector of desired angles - added constants relate to the
% equilibrium position and should not be changed
qf = [theta_1-pi/2,theta_2-2.08,theta_3+2.87,-theta_4+1.24,theta_5];

% If any of the desired angles are not possible based on the range of
% motion, this will trigger an error message
if qf(2)<-2.74||qf(2)>1.12||qf(3)<0||qf(3)>3.51||qf(4)<-.541||qf(4)>3.35
    quit = 1;
end

% Triggered when inverse kinematics produces joint angles that cannot be 
% reached by this robotic arm
if quit
    set(figure,'Color','white')
    plot(bounds_dist_from_center(1:end),bounds(1:end,3),'-r','LineWidth',2)
    hold on
    plot(point_dist_from_center,p(3),'r*','MarkerSize',8)
    title('End Effector Range')
    l= legend('end effector limits', 'desired position');
    set(l,'interpreter','latex','fontsize',10);
    set(gca,'fontsize',15)
    xlabel('distance from center axis ($\sqrt{x^2+y^2}$)','interpreter','latex','fontsize',20)
    ylabel('height (z)','interpreter','latex','fontsize',20)
    axis tight
    
    error('Desired position is inside the range of the robotic arm but cannot be reached. Try a different end effector axis orientation.');
end
