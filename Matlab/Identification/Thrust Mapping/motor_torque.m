clc
clear all
close all

num = xlsread('motor_torque_data.xlsx');
rpm = num(:,1);
mthrust = num(:,5)/2;
normalized_thrust = (mthrust - min(mthrust)) / ( max(mthrust) - min(mthrust));
plot(rpm,normalized_thrust);
%y = p1*x^2 + p2*x +  p3 
p = polyfit(normalized_thrust,rpm,2);
p(3)
