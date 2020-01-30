%% Generate identification data
%Uses the motor co-efficient data to generate motor rpm's for a given input

clear 
clc
close all

%forward thrust in range 0.0 to 0.6
%rotational thrust in range -0.4 to 0.4

%% parameters of id data
h_out = 0.05;            % sampling time of id data (check ros hz of mocap)
t_max = 31;             % time duration of id data  
t = 0:h_out:t_max;

%% beta hovercraft parameters
m = 0.1763;       % mass of hovercraft
Iz = 0.000761;     % moment of inertia around z axis
Xu = 0.09;        % linear damping
Yv = 0.08;
Nr = 0.001;     % yaw damping
l = 0.0425;       % lateral offset of thruster from center line

param = [m, l, Iz, Xu, Yv, Nr];

%% identification input
del_change_u = 2; %seconds after which you want to change input
del_change_r = 1;
u = idinput(length(t), 'prbs',[0,h_out/del_change_u],[-0.4,0.5]);
r = idinput(length(t), 'prbs',[0,h_out/del_change_r],[-0.3,0.3]); 

%% generate motor pwms

num = xlsread('motor_torque_data.xlsx');
rpm = num(:,1);
mthrust = num(:,5)/2;
normalized_thrust = (mthrust - min(mthrust)) / ( max(mthrust) - min(mthrust));
%plot(normalized_thrust,rpm);
p = polyfit(normalized_thrust,rpm,2);
pwmdataId.name = 'Identification PRBS Input Spec 1';
dataId.name = 'Identification PRBS Input Spec 1';
dataId.t = t;
dataId.u = u;
dataId.r = r;
dataId.param = param;
dataId.h = h_out;

for i = 1:length(t)-1
    pwmdataId.t(i) = t(i);
    pwmdataId.PWM(:,i) = generatePWM(u(i),r(i),p);
end
save('input_data_id_spec1','dataId');
save('pwmdataId_spec_1','pwmdataId');



