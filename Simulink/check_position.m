function [bounds,bounds_dist_from_center,point_dist_from_center] = check_position(p)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function checks the desired end effector position to ensure it is
% within the range of motion of the WidowX 200 robotic arm. If it is not
% within the range, an error message is displayed and the program
% terminated. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Import data specifying the range of the end effector
filename = 'simulink_model_bounds.mat';
load(filename);

% Range is axially symmetric about the z-axis, so the distance from the
% z-axis is calculated to determine the range
bounds_dist_from_center = sqrt((bounds(:,1)).^2+(bounds(:,2)).^2);
point_dist_from_center = sqrt((p(1))^2+(p(2))^2);

% Checks if desired end effector position is within range
[in,on] = inpolygon(point_dist_from_center,p(3),bounds_dist_from_center(1:end),bounds(1:end,3));

if in==0&&on==0
    
    % If position is outside the possible range, display error and plot in
    % red
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
    
    error('Desired position cannot be reached, because it is outside the range of motion of the robotic arm.');
else
    
    [in,on] = inpolygon(p(1),p(2),microcontroller_bounds(1:end,1),microcontroller_bounds(1:end,2));
    
    
    % If position may intersect with the microcontroller box, check height
    if (in==1||on==1)&&p(3)<.05
        
        % Display error and plot in red if positions is within
        % mircocontroller box
        bounds_dist_from_center_mc = sqrt((bounds_with_microcontroller(:,1)).^2+(bounds_with_microcontroller(:,2)).^2);
        
        set(figure,'Color','white')
        plot(bounds_dist_from_center_mc(1:end),bounds_with_microcontroller(1:end,3),'-r','LineWidth',2)
        hold on
        plot(point_dist_from_center,p(3),'r*','MarkerSize',8)
        title('End Effector Range')
        l= legend('end effector limits', 'desired position');
        set(l,'interpreter','latex','fontsize',10);
        set(gca,'fontsize',15)
        xlabel('distance from center axis ($\sqrt{x^2+y^2}$)','interpreter','latex','fontsize',20)
        ylabel('height (z)','interpreter','latex','fontsize',20)
        axis tight
    
        error('Desired position cannot be reached, because it is outside the range of motion of the robotic arm. Watch out for the microcontroller box.');
    else
        
        % Check distance to the end of the range
        dist = sqrt((point_dist_from_center - [0;bounds_dist_from_center(1:70)]).^2+(p(3)-[.69;bounds(1:70,3)]).^2);
        
        % If the position is near the end of the range, plot in orange and
        % display warning message
        if min(dist)<.05
            
            warning('Desired end effector position is near the limits of the range and the arm may have difficulty reaching it.');

            set(figure,'Color','white')
            plot(bounds_dist_from_center(1:end),bounds(1:end,3),'-','color',[245 115 0]/255,'LineWidth',2)
            hold on
            plot(point_dist_from_center,p(3),'*','color',[245 115 0]/255,'MarkerSize',8)
            title('End Effector Range')
            l= legend('end effector limits', 'desired position');
            set(l,'interpreter','latex','fontsize',10);
            set(gca,'fontsize',15)
            xlabel('distance from center axis ($\sqrt{x^2+y^2}$)','interpreter','latex','fontsize',20)
            ylabel('height (z)','interpreter','latex','fontsize',20)
            axis tight
            
        else
        
            % If none of the other conditions were met, the position is
            % well within range and plots in blue
            set(figure,'Color','white')
            plot(bounds_dist_from_center(1:end),bounds(1:end,3),'-b','LineWidth',2)
            hold on
            plot(point_dist_from_center,p(3),'b*','MarkerSize',8)
            title('End Effector Range')
            l= legend('end effector limits', 'desired position');
            set(l,'interpreter','latex','fontsize',10);
            set(gca,'fontsize',15)
            xlabel('distance from center axis ($\sqrt{x^2+y^2}$)','interpreter','latex','fontsize',20)
            ylabel('height (z)','interpreter','latex','fontsize',20)
            axis tight
        
        end 
    end
end


