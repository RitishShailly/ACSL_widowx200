function [current_input] = torque_to_current_input(tor)

%slope of the current-torque relation
torque_slope = (1.8-0.12)/(2.975-0.07);
%y - intercept
offset = 0.12-0.07*torque_slope;

%current level
current_intermediate = (tor*torque_slope)+offset; 
%current quantization
current_level = current_intermediate/(2.69*10^-3);
%current command
current_input = round(current_level);
 
end

