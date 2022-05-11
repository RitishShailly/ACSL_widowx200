%%
clc
clear all
close all

addpath(genpath('~/DynamixelSDK-3.7.21'));

[num, txt, raw] = xlsread('real_robotic_arm_parameters.xlsx');
omega = num(:,1);
zeta = num(:,2);
Ki = num(:,3);
r11 = num(:,4);
r22 = num(:,5);
r2 = num(:,6);
theta = num(:,7);

%Define the number of degrees of freedom
n_dof = 5;



lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end


ADDR_PRO_TORQUE_ENABLE          = 64; %562;          % Control table address is different in Dynamixel model
ADDR_PRO_LED_RED                = 65; %563;
ADDR_PRO_GOAL_POSITION          = 116;%596;
ADDR_PRO_PRESENT_POSITION       = 132;%611;


% Data Byte LengthFuture Technology Devices International, Ltd FT232H Single HS USB-UART/FIFO IC

LEN_PRO_GOAL_POSITION       = 4;
LEN_PRO_PRESENT_POSITION    = 4;

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL1_ID                     = 1;            % Dynamixel#1 ID: 1
DXL2_ID                     = 2;            % Dynamixel#2 ID: 2
BAUDRATE                    = 1000000;
DEVICENAME                  = '/dev/ttyUSB0';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

assignin('base','port_num',port_num);

% Initialize PacketHandler Structs
packetHandler();

%Initialize Groupsyncwrite Structs
groupwrite_current = groupSyncWrite(port_num, PROTOCOL_VERSION, 102,2);

% Initialize Groupsyncread Structs for Present Position
groupread_position = groupSyncRead(port_num, PROTOCOL_VERSION, 132,4);
groupread_velocity = groupSyncRead(port_num, PROTOCOL_VERSION, 128,4);

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_addparam_result = false;              % AddParam result
dxl_getdata_result = false;               % GetParam result

dxl_error = 0;                              % Dynamixel error
dxl1_present_position = 0;                  % Present position
dxl2_present_position = 0;


% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

dxl_addparam_result = groupSyncReadAddParam(groupread_velocity, 1);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end


dxl_addparam_result = groupSyncReadAddParam(groupread_velocity, 2);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end

dxl_addparam_result = groupSyncReadAddParam(groupread_velocity, 4);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end


dxl_addparam_result = groupSyncReadAddParam(groupread_velocity, 5);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end

dxl_addparam_result = groupSyncReadAddParam(groupread_velocity, 6);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end





dxl_addparam_pos_result = groupSyncReadAddParam(groupread_position, 1);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end



dxl_addparam_pos_result = groupSyncReadAddParam(groupread_position, 2);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end



dxl_addparam_pos_result = groupSyncReadAddParam(groupread_position, 4);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end


dxl_addparam_pos_result = groupSyncReadAddParam(groupread_position, 5);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end

dxl_addparam_pos_result = groupSyncReadAddParam(groupread_position, 6);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncRead addparam failed', DXL1_ID);
  return;
end


m1 = 1.607196;
m2 = 0.715452;
m3 = 0.669306;
m4 = 0.416434;
m5 = 0.730841;

% Define the gravitational acceleration [m/s^2]
g = 9.81;

% Define the length of each link [m]

l1 = 0.11325;
l2 = 0.20616;
l3 = 0.2;
l4 = 0.065;
l5 = 0.10915;


% Define the moment of inertia for each link w.r.t. its center of mass

I1 = 0.002175;
I2 = 0.00401991;
I3 = 0.00258893;
I4 = 0.00013472;
I5 = 0.00029738;

lc1 = 0.047;
lc2 = 0.1353;
lc3 = 0.118;
lc4 = 0.0444;
lc5 = 0.0955;


t2rconv = 2*pi/4095;   %ticks to radians conversion
velconv = 0.0240;      %velocity conversion from rpm to rads/sec 

qref_plot = zeros(5,1);
qrefdot_plot = zeros(5,1);
qrefddot_plot = zeros(5,1);

plot_v_diff = [];
torque_v = [];
plot_q = zeros(5,1);
plot_v = [];
p =[];
q_check =[];
error_plot = [];

%xl.sync_write_four([1,2,3,4],38,[1193,1193,1193,1193],2,s);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 64, 0); %TORQUE OFF
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 64, 0); %TORQUE OFF
% 

% write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, 11, 3);

write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, 64, 0);

 write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, 11, 0);

%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 3, 11, 3);
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, 11, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, 64, 1);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 3, 64, 1);

% write1ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_PRO_TORQUE_ENABLE,1);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_PRO_TORQUE_ENABLE,1);

% write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, 11, 3);%CONTROL MODE : POSITION
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, 64, 1); %TORQUE ON 
% write4ByteTxRx(port_num, PROTOCOL_VERSION, 1, 116, 2090);

pause(1)

write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, 64, 0); %TORQUE OFF
write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, 11, 0);%CONTROL MODE : current
write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, 64, 1); %TORQUE ON

% write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 64, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 11, 3);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 64, 1);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, 5, 116, 2048);

pause(01)

% write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, 11, 3);%CONTROL MODE : POSITION
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, 64, 1); %TORQUE ON

% 
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 64, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 11, 3);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 64, 1);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, 4, 116, 1780);



write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 64, 0); %TORQUE OFF
write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 11, 0);%CONTROL MODE : current
write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 64, 1);

write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 64, 0); %TORQUE OFF
write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 11, 0);%CONTROL MODE : current
write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 64, 1);

write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 64, 0); %TORQUE OFF
write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 11, 3);%CONTROL MODE : current
write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, 64, 1);


command_plot = [];




omega_link1 = omega(1);          % Must be larger than zero - See notes on PD tuning
omega_link2 = omega(2);          % Must be larger than zero - See notes on PD tuning
omega_link3 = omega(3);          % Must be larger than zero - See notes on PD tuning
omega_link4 = omega(4);          % Must be larger than zero - See notes on PD tuning         
omega_link5 = omega(5);          % Must be larger than zero - See notes on PD tuning

zeta_link1 = zeta(1);          % Must be larger than zero - See notes on PD tuning
zeta_link2 = zeta(2);          % Must be larger than zero - See notes on PD tuning
zeta_link3 = zeta(3);          % Must be larger than zero - See notes on PD tuning
zeta_link4 = zeta(4);          % Must be larger than zero - See notes on PD tuning
zeta_link5 = zeta(5);          % Must be larger than zero - See notes on PD tuning



omega = [omega_link1 ,omega_link2,omega_link3,omega_link4,omega_link5];
omega= diag(omega);

zeta = [zeta_link1,zeta_link2,zeta_link3,zeta_link4,zeta_link5];
zeta = diag(zeta);

Ki = diag(Ki);
%use over damped system

% Parameters for the LQR controller
r11_link1 = r11(1);           % Must be positive
r22_link1 = r22(1);           % Must be positive
R1_link1 = [r11_link1,0;0,r22_link1];
r2_link1 = r2(1);           % Must be positive

r11_link2 = r11(2);           % Must be positive
r22_link2 = r22(2);           % Must be positive
R1_link2 = [r11_link2,0;0,r22_link2];
r2_link2 = r2(2);           % Must be positive

r11_link3 = r11(3);           % Must be positive
r22_link3 = r22(3);           % Must be positive
R1_link3 = [r11_link3,0;0,r22_link3];
r2_link3 = r2(3);             % Must be positive

r11_link4 = r11(4);           % Must be positive
r22_link4 = r22(4);           % Must be positive
R1_link4 = [r11_link4,0;0,r22_link4];
r2_link4 = r2(4);             % Must be positive

r11_link5 = r11(5);           % Must be positive
r22_link5 = r22(5);           % Must be positive
R1_link5 = [r11_link5,0;0,r22_link5];
r2_link5 = r2(5);             % Must be positive




A1 = [0,1;-2*zeta_link1*omega_link1,-omega_link1^2]; % A matrix for link 1

A2 = [0,1;-2*zeta_link2*omega_link2,-omega_link2^2];

A3 = [0,1;-2*zeta_link3*omega_link3,-omega_link3^2];

A4 = [0,1;-2*zeta_link4*omega_link4,-omega_link4^2];

A5 = [0,1;-2*zeta_link5*omega_link5,-omega_link5^2];


B = [0;1];
C11 = eye(2);
D = [0;0];

sys1 = ss(A1,B,C11,D); % Matlab representation of the first link's dynamics
sys2 = ss(A2,B,C11,D);
sys3 = ss(A3,B,C11,D);
sys4 = ss(A4,B,C11,D);
sys5 = ss(A5,B,C11,D);


[K_LQR1,~,~] = lqr(sys1,R1_link1,r2_link1,zeros(2,1));
[K_LQR2,~,~] = lqr(sys2,R1_link2,r2_link2,zeros(2,1));
[K_LQR3,~,~] = lqr(sys3,R1_link3,r2_link3,zeros(2,1));
[K_LQR4,~,~] = lqr(sys4,R1_link4,r2_link4,zeros(2,1));
[K_LQR5,~,~] = lqr(sys5,R1_link5,r2_link5,zeros(2,1));




% Kp = omega_link1^2;
Kp = omega.^2;

% Kd = 2*zeta_link1(1)*omega_link1;

Kd = 2*omega*zeta;

%Kd(3,3) = 04.25 ;

A3check = [0 1 ; -2*zeta_link3*omega_link3-K_LQR3(1),-omega_link3^2-K_LQR3(2)];
eig(A3check);



integral_plot=[];


 groupSyncReadTxRxPacket(groupread_position);
    
     %read position
    
    q1 = double(typecast(uint32( groupSyncReadGetData(groupread_position, 1,132, 4)),'int32'));
   
    q2 = double(typecast(uint32(groupSyncReadGetData(groupread_position, 2,132, 4)),'int32'));
    
    q3 = double(typecast(uint32(groupSyncReadGetData(groupread_position, 4,132, 4)),'int32'));
    
    q4 = double(typecast(uint32(groupSyncReadGetData(groupread_position, 5,132, 4)),'int32'));
 
    q5 = double(typecast(uint32(groupSyncReadGetData(groupread_position, 6,132, 4)),'int32'));
 
 
    q0 =[q1;q2;q3;q4;q5]; %add 6th motor
    
    
     q0 = t2rconv*q0';  %converto to radians
    
    q0 = q0-pi;
    
    q0 = q0';
    
    q = q0;
    
    q=q';
    
    
    tf = 2000;
    
    t1 = linspace(0,tf-1,tf); 
    q_ref_t = zeros(length(t1),length(q0));
    q_ref_dot_t = zeros(length(t1),length(q0));
    q_ref_ddot_t = zeros(length(t1),length(q0));

%     qf =  [-0.0836  ; -0.98 ;  -0.9  ; pi/4  ; -0.0253]; %final
%     position

    qf = theta;
    %qf =  [ pi/4  ; -0.5 ;  -0.5  ; 0.5 ; -0.0253];
     
     
     
    for jj = 1:length(t1)
    [q_ref_t(jj,:),q_ref_dot_t(jj,:),q_ref_ddot_t(jj,:)] = LSPB(tf,q0,qf,t1(jj));
    end
    

    q_ref_t = q_ref_t';
   
    q_ref_dot_t =q_ref_dot_t';
    
    q_ref_ddot_t = q_ref_ddot_t';

timediff =[0];

pos_5 = qf(5);

pos_5 = pos_5/t2rconv;

write4ByteTxRx(port_num, PROTOCOL_VERSION, 6, 116, pos_5);
  
tic
for t=1:length(t1) %change the variable t 
    
    
     q0 = q';

    groupSyncReadTxRxPacket(groupread_position)
    time_1 = clock;
    time_1 = time_1(6);
     %read position
    
    q1 = double(typecast(uint32( groupSyncReadGetData(groupread_position, 1,132, 4)),'int32'));
   
    q2 = double(typecast(uint32(groupSyncReadGetData(groupread_position, 2,132, 4)),'int32'));
    
    q3 = double(typecast(uint32(groupSyncReadGetData(groupread_position, 4,132, 4)),'int32'));
    
    q4 = double(typecast(uint32(groupSyncReadGetData(groupread_position, 5,132, 4)),'int32'));
 
    q5 = double(typecast(uint32(groupSyncReadGetData(groupread_position, 6,132, 4)),'int32'));
 
 

    q =[q1;q2;q3;q4;q5]; 
    
    


%     q(q<0) = q(q<0) +4095; %convert to positive
% 
%     q = mod(q,4096);  %check for remainder over 4095 bits 
%     
%     q(q<2048) = q(q<2048)-4095;
%     q(q>2047) = 4095-q(q>2047);
    q = t2rconv*q';  %converto to radians
    
    q = q-pi;
    
    q_dot = (q'-q0)/(time_1-timediff(1,end));

    timediff = [timediff time_1];

 
    plot_v = [plot_v q_dot];
    
   q_dot1 = q_dot(1);
   q_dot2 = q_dot(2);
   q_dot3 = q_dot(3);
   q_dot4 = q_dot(4);
   q_dot5 = q_dot(5);
    

    plot_q = [plot_q,q' ];  %plotting positions for 1st joint  
    
    
%   reference_trajectory; 

   qref = q_ref_t(:,t);
   qrefdot =  q_ref_dot_t (:,t);
   qrefddot = q_ref_ddot_t(:,t);

    qref_plot = [qref_plot,qref];
    qrefdot_plot = [qrefdot_plot,qrefdot ];
    qrefddot_plot = [qrefddot_plot ,qrefddot];
    
    
    e = q'-qref;              %position error
    e_dot = q_dot-qrefdot;   %velocity error
    

    
    error_plot = [error_plot e];
    
    if t==1
        e_int = zeros(5,1);
    else 
        e_int = trapz(error_plot');
        e_int = e_int';
    end


    integral_plot = [integral_plot e_int];
    
    
     M12 = [ I1 + I2 + I3 + I4 + I5 + m4*cos(q(1))^2*(l3*cos(q(2) + q(3)) + l2*cos(q(2)) + lc4*cos(q(2) + q(3) + q(4)))^2 + m5*cos(q(1))^2*(l3*cos(q(2) + q(3)) + l2*cos(q(2)) + lc5*cos(q(2) + q(3) + q(4)))^2 + m4*sin(q(1))^2*(l3*cos(q(2) + q(3)) + l2*cos(q(2)) + lc4*cos(q(2) + q(3) + q(4)))^2 + m5*sin(q(1))^2*(l3*cos(q(2) + q(3)) + l2*cos(q(2)) + lc5*cos(q(2) + q(3) + q(4)))^2 + m3*cos(q(1))^2*(lc3*cos(q(2) + q(3)) + l2*cos(q(2)))^2 + m3*sin(q(1))^2*(lc3*cos(q(2) + q(3)) + l2*cos(q(2)))^2 + lc2^2*m2*cos(q(1))^2*cos(q(2))^2 + lc2^2*m2*cos(q(2))^2*sin(q(1))^2,                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                         0,                                                                                                                       0, -I5*cos(q(2) + q(3) + q(4));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      0, I2 + I3 + I4 + I5 + l2^2*m3 + l2^2*m4 + l2^2*m5 + l3^2*m4 + l3^2*m5 + lc2^2*m2 + lc3^2*m3 + lc4^2*m4 + lc5^2*m5 + 2*l2*lc4*m4*cos(q(3) + q(4)) + 2*l2*lc5*m5*cos(q(3) + q(4)) + 2*l2*l3*m4*cos(q(3)) + 2*l2*l3*m5*cos(q(3)) + 2*l2*lc3*m3*cos(q(3)) + 2*l3*lc4*m4*cos(q(4)) + 2*l3*lc5*m5*cos(q(4)), I3 + I4 + I5 + l3^2*m4 + l3^2*m5 + lc3^2*m3 + lc4^2*m4 + lc5^2*m5 + l2*lc4*m4*cos(q(3) + q(4)) + l2*lc5*m5*cos(q(3) + q(4)) + l2*l3*m4*cos(q(3)) + l2*l3*m5*cos(q(3)) + l2*lc3*m3*cos(q(3)) + 2*l3*lc4*m4*cos(q(4)) + 2*l3*lc5*m5*cos(q(4)), I4 + I5 + lc4^2*m4 + lc5^2*m5 + l2*lc4*m4*cos(q(3) + q(4)) + l2*lc5*m5*cos(q(3) + q(4)) + l3*lc4*m4*cos(q(4)) + l3*lc5*m5*cos(q(4)),                     0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      0,                                                         I3 + I4 + I5 + l3^2*m4 + l3^2*m5 + lc3^2*m3 + lc4^2*m4 + lc5^2*m5 + l2*lc4*m4*cos(q(3) + q(4)) + l2*lc5*m5*cos(q(3) + q(4)) + l2*l3*m4*cos(q(3)) + l2*l3*m5*cos(q(3)) + l2*lc3*m3*cos(q(3)) + 2*l3*lc4*m4*cos(q(4)) + 2*l3*lc5*m5*cos(q(4)),                                                                                                             I3 + I4 + I5 + l3^2*m4 + l3^2*m5 + lc3^2*m3 + lc4^2*m4 + lc5^2*m5 + 2*l3*lc4*m4*cos(q(4)) + 2*l3*lc5*m5*cos(q(4)),                                                   m4*lc4^2 + l3*m4*cos(q(4))*lc4 + m5*lc5^2 + l3*m5*cos(q(4))*lc5 + I4 + I5,                     0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      0,                                                                                                                                                           I4 + I5 + lc4^2*m4 + lc5^2*m5 + l2*lc4*m4*cos(q(3) + q(4)) + l2*lc5*m5*cos(q(3) + q(4)) + l3*lc4*m4*cos(q(4)) + l3*lc5*m5*cos(q(4)),                                                                                                                                                     m4*lc4^2 + l3*m4*cos(q(4))*lc4 + m5*lc5^2 + l3*m5*cos(q(4))*lc5 + I4 + I5,                                                                                           m4*lc4^2 + m5*lc5^2 + I4 + I5,                     0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                  -I5*cos(q(2) + q(3) + q(4)),                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                         0,                                                                                                                       0,                    I5];
 

    c1 = [(- (l3^2*m4*sin(2*q(2) + 2*q(3)))/2 - (l3^2*m5*sin(2*q(2) + 2*q(3)))/2 - (lc3^2*m3*sin(2*q(2) + 2*q(3)))/2 - (l2^2*m3*sin(2*q(2)))/2 - (l2^2*m4*sin(2*q(2)))/2 - (l2^2*m5*sin(2*q(2)))/2 - (lc2^2*m2*sin(2*q(2)))/2 - (lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - l2*lc4*m4*sin(2*q(2) + q(3) + q(4)) - l2*lc5*m5*sin(2*q(2) + q(3) + q(4)) - l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)) - l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)) - l2*l3*m4*sin(2*q(2) + q(3)) - l2*l3*m5*sin(2*q(2) + q(3)) - l2*lc3*m3*sin(2*q(2) + q(3)))*q_dot(2) + (- (l3^2*m4*sin(2*q(2) + 2*q(3)))/2 - (l3^2*m5*sin(2*q(2) + 2*q(3)))/2 - (lc3^2*m3*sin(2*q(2) + 2*q(3)))/2 - (lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (l2*lc4*m4*sin(2*q(2) + q(3) + q(4)))/2 - (l2*lc5*m5*sin(2*q(2) + q(3) + q(4)))/2 - (l2*lc4*m4*sin(q(3) + q(4)))/2 - (l2*lc5*m5*sin(q(3) + q(4)))/2 - (l2*l3*m4*sin(q(3)))/2 - (l2*l3*m5*sin(q(3)))/2 - (l2*lc3*m3*sin(q(3)))/2 - l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)) - l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)) - (l2*l3*m4*sin(2*q(2) + q(3)))/2 - (l2*l3*m5*sin(2*q(2) + q(3)))/2 - (l2*lc3*m3*sin(2*q(2) + q(3)))/2)*q_dot(3) + (- (lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (l2*lc4*m4*sin(2*q(2) + q(3) + q(4)))/2 - (l2*lc5*m5*sin(2*q(2) + q(3) + q(4)))/2 - (l2*lc4*m4*sin(q(3) + q(4)))/2 - (l2*lc5*m5*sin(q(3) + q(4)))/2 - (l3*lc4*m4*sin(q(4)))/2 - (l3*lc5*m5*sin(q(4)))/2 - (l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)))/2 - (l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)))/2)*q_dot(4), (- (l3^2*m4*sin(2*q(2) + 2*q(3)))/2 - (l3^2*m5*sin(2*q(2) + 2*q(3)))/2 - (lc3^2*m3*sin(2*q(2) + 2*q(3)))/2 - (l2^2*m3*sin(2*q(2)))/2 - (l2^2*m4*sin(2*q(2)))/2 - (l2^2*m5*sin(2*q(2)))/2 - (lc2^2*m2*sin(2*q(2)))/2 - (lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - l2*lc4*m4*sin(2*q(2) + q(3) + q(4)) - l2*lc5*m5*sin(2*q(2) + q(3) + q(4)) - l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)) - l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)) - l2*l3*m4*sin(2*q(2) + q(3)) - l2*l3*m5*sin(2*q(2) + q(3)) - l2*lc3*m3*sin(2*q(2) + q(3)))*q_dot(1) + ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(5), (- (l3^2*m4*sin(2*q(2) + 2*q(3)))/2 - (l3^2*m5*sin(2*q(2) + 2*q(3)))/2 - (lc3^2*m3*sin(2*q(2) + 2*q(3)))/2 - (lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (l2*lc4*m4*sin(2*q(2) + q(3) + q(4)))/2 - (l2*lc5*m5*sin(2*q(2) + q(3) + q(4)))/2 - (l2*lc4*m4*sin(q(3) + q(4)))/2 - (l2*lc5*m5*sin(q(3) + q(4)))/2 - (l2*l3*m4*sin(q(3)))/2 - (l2*l3*m5*sin(q(3)))/2 - (l2*lc3*m3*sin(q(3)))/2 - l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)) - l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)) - (l2*l3*m4*sin(2*q(2) + q(3)))/2 - (l2*l3*m5*sin(2*q(2) + q(3)))/2 - (l2*lc3*m3*sin(2*q(2) + q(3)))/2)*q_dot(1) + ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(5), (- (lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 - (l2*lc4*m4*sin(2*q(2) + q(3) + q(4)))/2 - (l2*lc5*m5*sin(2*q(2) + q(3) + q(4)))/2 - (l2*lc4*m4*sin(q(3) + q(4)))/2 - (l2*lc5*m5*sin(q(3) + q(4)))/2 - (l3*lc4*m4*sin(q(4)))/2 - (l3*lc5*m5*sin(q(4)))/2 - (l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)))/2 - (l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)))/2)*q_dot(1) + ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(5), ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(2) + ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(3) + ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(4);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  ((l3^2*m4*sin(2*q(2) + 2*q(3)))/2 + (l3^2*m5*sin(2*q(2) + 2*q(3)))/2 + (lc3^2*m3*sin(2*q(2) + 2*q(3)))/2 + (l2^2*m3*sin(2*q(2)))/2 + (l2^2*m4*sin(2*q(2)))/2 + (l2^2*m5*sin(2*q(2)))/2 + (lc2^2*m2*sin(2*q(2)))/2 + (lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 + (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 + l2*lc4*m4*sin(2*q(2) + q(3) + q(4)) + l2*lc5*m5*sin(2*q(2) + q(3) + q(4)) + l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)) + l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)) + l2*l3*m4*sin(2*q(2) + q(3)) + l2*l3*m5*sin(2*q(2) + q(3)) + l2*lc3*m3*sin(2*q(2) + q(3)))*q_dot(1) + (-(I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(5),                                                                                                                                                                                                                                                                                                                                                                       (-l2*(l3*m4*sin(q(3)) + l3*m5*sin(q(3)) + lc3*m3*sin(q(3)) + lc4*m4*sin(q(3) + q(4)) + lc5*m5*sin(q(3) + q(4))))*q_dot(3) + (-(l2*sin(q(3) + q(4)) + l3*sin(q(4)))*(lc4*m4 + lc5*m5))*q_dot(4),                                                                                                                                                                                                                                                                                                              (-l2*(l3*m4*sin(q(3)) + l3*m5*sin(q(3)) + lc3*m3*sin(q(3)) + lc4*m4*sin(q(3) + q(4)) + lc5*m5*sin(q(3) + q(4))))*q_dot(2) + (-l2*(l3*m4*sin(q(3)) + l3*m5*sin(q(3)) + lc3*m3*sin(q(3)) + lc4*m4*sin(q(3) + q(4)) + lc5*m5*sin(q(3) + q(4))))*q_dot(3) + (-(l2*sin(q(3) + q(4)) + l3*sin(q(4)))*(lc4*m4 + lc5*m5))*q_dot(4),                                                                                                                                                                                                     (-(l2*sin(q(3) + q(4)) + l3*sin(q(4)))*(lc4*m4 + lc5*m5))*q_dot(2) + (-(l2*sin(q(3) + q(4)) + l3*sin(q(4)))*(lc4*m4 + lc5*m5))*q_dot(3) + (-(l2*sin(q(3) + q(4)) + l3*sin(q(4)))*(lc4*m4 + lc5*m5))*q_dot(4),                                                                        (-(I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(1);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ((l3^2*m4*sin(2*q(2) + 2*q(3)))/2 + (l3^2*m5*sin(2*q(2) + 2*q(3)))/2 + (lc3^2*m3*sin(2*q(2) + 2*q(3)))/2 + (lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 + (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 + (l2*lc4*m4*sin(2*q(2) + q(3) + q(4)))/2 + (l2*lc5*m5*sin(2*q(2) + q(3) + q(4)))/2 + (l2*lc4*m4*sin(q(3) + q(4)))/2 + (l2*lc5*m5*sin(q(3) + q(4)))/2 + (l2*l3*m4*sin(q(3)))/2 + (l2*l3*m5*sin(q(3)))/2 + (l2*lc3*m3*sin(q(3)))/2 + l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)) + l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)) + (l2*l3*m4*sin(2*q(2) + q(3)))/2 + (l2*l3*m5*sin(2*q(2) + q(3)))/2 + (l2*lc3*m3*sin(2*q(2) + q(3)))/2)*q_dot(1) + (-(I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(5),                                                                                                                                                                                                                                                                                                                                                                                              l2*(l3*m4*sin(q(3)) + l3*m5*sin(q(3)) + lc3*m3*sin(q(3)) + lc4*m4*sin(q(3) + q(4)) + lc5*m5*sin(q(3) + q(4)))*q_dot(2) + (-l3*sin(q(4))*(lc4*m4 + lc5*m5))*q_dot(4),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          (-l3*sin(q(4))*(lc4*m4 + lc5*m5))*q_dot(4),                                                                                                                                                                                                                                                                 (-l3*sin(q(4))*(lc4*m4 + lc5*m5))*q_dot(2) + (-l3*sin(q(4))*(lc4*m4 + lc5*m5))*q_dot(3) + (-l3*sin(q(4))*(lc4*m4 + lc5*m5))*q_dot(4),                                                                        (-(I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(1);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       ((lc4^2*m4*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 + (lc5^2*m5*sin(2*q(2) + 2*q(3) + 2*q(4)))/2 + (l2*lc4*m4*sin(2*q(2) + q(3) + q(4)))/2 + (l2*lc5*m5*sin(2*q(2) + q(3) + q(4)))/2 + (l2*lc4*m4*sin(q(3) + q(4)))/2 + (l2*lc5*m5*sin(q(3) + q(4)))/2 + (l3*lc4*m4*sin(q(4)))/2 + (l3*lc5*m5*sin(q(4)))/2 + (l3*lc4*m4*sin(2*q(2) + 2*q(3) + q(4)))/2 + (l3*lc5*m5*sin(2*q(2) + 2*q(3) + q(4)))/2)*q_dot(1) + (-(I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(5),                                                                                                                                                                                                                                                                                                                                                                                                                                                (l2*sin(q(3) + q(4)) + l3*sin(q(4)))*(lc4*m4 + lc5*m5)*q_dot(2) + l3*sin(q(4))*(lc4*m4 + lc5*m5)*q_dot(3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       l3*sin(q(4))*(lc4*m4 + lc5*m5)*q_dot(2) + l3*sin(q(4))*(lc4*m4 + lc5*m5)*q_dot(3),                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                        (-(I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(1);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(2) + ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(3) + ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(4),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(1),                                                                                                                                                                                                                                                                                                                                                        ((I5*sin(q(2) + q(3) + q(4)))/2)*q_dot(1),                                                                                                         0];
    Kq1 = [                                                                                                                                                                                        0;
     g*(l2*m3*cos(q(2)) + l2*m4*cos(q(2)) + l2*m5*cos(q(2)) + lc2*m2*cos(q(2)) + lc4*m4*cos(q(2) + q(3) + q(4)) + lc5*m5*cos(q(2) + q(3) + q(4)) + l3*m4*cos(q(2) + q(3)) + l3*m5*cos(q(2) + q(3)) + lc3*m3*cos(q(2) + q(3)));
                                                                  g*(lc4*m4*cos(q(2) + q(3) + q(4)) + lc5*m5*cos(q(2) + q(3) + q(4)) + l3*m4*cos(q(2) + q(3)) + l3*m5*cos(q(2) + q(3)) + lc3*m3*cos(q(2) + q(3)));
                                                                                                                                                    g*cos(q(2) + q(3) + q(4))*(lc4*m4 + lc5*m5);
                                                                                                                                                                                        0];
 
    
    
      


     u_LQR1 = -K_LQR1*[e(1);e_dot(1)];   % u from LQR
     u_LQR2 = -K_LQR2*[e(2);e_dot(2)];
     u_LQR3 = -K_LQR3*[e(3);e_dot(3)];
     u_LQR4 = -K_LQR4*[e(4);e_dot(4)];
     u_LQR5 = -K_LQR5*[e(5);e_dot(5)];
     
     u_LQR =[u_LQR1;u_LQR2;u_LQR3;u_LQR4;u_LQR5];
     
     
     torque = M12*(qrefddot - Kp*e - Kd*e_dot - Ki*e_int) + c1*q_dot + Kq1 ; %u baseline 
      
     current_command_lqr = torque_to_current_input(u_LQR); %current command for torque from lqr

     current_command = torque_to_current_input(torque);     %current command for torque from baseline
   
     
     
     total_command = current_command+current_command_lqr; %total current command
     
    
     %limit command to 1050
     
     total_command(total_command < -1050) = -1050;
     total_command(total_command > 1050 ) = 1050;
%      

%      

%  total_command(2)

     
    dxl_write = groupSyncWriteAddParam(groupwrite_current,1, typecast(int16(total_command(1)), 'uint16'),2);
    dxl_write = groupSyncWriteAddParam(groupwrite_current,2, typecast(int16(total_command(2)), 'uint16'),2);
    dxl_write = groupSyncWriteAddParam(groupwrite_current, 4 , typecast(int16(total_command(3)), 'uint16'),2);
    dxl_write = groupSyncWriteAddParam(groupwrite_current,5 , typecast(int16(total_command(4)), 'uint16'),2);
    
     groupSyncWriteTxPacket(groupwrite_current);
     groupSyncWriteClearParam(groupwrite_current);
     

   
end
total_time =toc;
time_loop = total_time/t;
freq = 1/time_loop
p = linspace(0,t-1,t);
p = p*time_loop;




%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_PRO_TORQUE_ENABLE, 0);
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_PRO_TORQUE_ENABLE, 0);
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_PRO_TORQUE_ENABLE, 0);
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_PRO_TORQUE_ENABLE, 0);
% 
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, 11, 3);
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, 11, 3);
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, 11, 3); 
  
 

% write1ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_PRO_TORQUE_ENABLE, 0)
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_PRO_TORQUE_ENABLE, 0);
%  write1ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_PRO_TORQUE_ENABLE, 0);
% plot(integral_plot)

%% Sending data to excel file

p_table_data = table(p);
qref_plot_table = table(qref_plot(:,2:t+1));
plot_q_table = table(plot_q(:,2:end));

writetable(p_table_data, '~/interbotix_ws/src/Interbotix_src/Actual_robot/p_data.xlsx');
writetable(qref_plot_table, '~/interbotix_ws/src/Interbotix_src/Actual_robot/qref_plot_data.xlsx');
writetable(plot_q_table, '~/interbotix_ws/src/Interbotix_src/Actual_robot/plot_q_data.xlsx');

%, qref_plot(1,2:t+1),plot_q(1,2:end), ...
%                    qref_plot(2,2:t+1),plot_q(2,2:end),...
%                    qref_plot(3,2:t+1),plot_q(3,2:end),...
%                    qref_plot(4,2:t+1),plot_q(4,2:end),...
%                    qref_plot(5,2:t+1),plot_q(5,2:end))
% file_name = 'real_robotic_arm_plot_data.xlsx';
% writetable(table_data,file_name)

%% Plots

%plot(p,qref_plot(1,2:t+1),p,plot_q(1,2:end),'LineWidth',2)
% hold on
%plot(p,qref_plot(2,2:t+1),p,plot_q(2,2:end),'LineWidth',2)
%hold on
% plot(p,qref_plot(3,2:t+1),p,plot_q(3,2:end),'LineWidth',2)
%  hold on
% plot(p,qref_plot(4,2:t+1),p,plot_q(4,2:end),'LineWidth',2)
% hold on
% plot(p,qref_plot(5,2:t+1),p,plot_q(5,2:end),'LineWidth',2)
 
% % plot(p,qref_plot(1,2:t+1),p,plot_q(1,2:end),p,error_plot(1,:),'LineWidth',2)  %plot of actual vs ref trajectory 
%grid on
%title('link2')
%xlabel('seconds')
%ylabel('angle in radians')
%legend('reference 1', 'actual 1', 'reference 2', 'actual 2 ','reference 3', 'actual 3', 'reference 4', 'actual 4')


%%
%Disconnect
 closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

%close all;
