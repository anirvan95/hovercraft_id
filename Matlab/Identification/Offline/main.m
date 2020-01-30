%% Hovercraft parameter identification - Optimization based
%  main program for fitting parameters of ODE hovercraft model to data
%  objective function is defined in objFun.m
%  dynamic model is defined in dynamicsId.m
%
clc
clear all
close all

load('spec2_2_data_id.mat')

%% Initial parameters for model
%Measured 
m = 0.1763;
l = 0.0425;

%Estimated
Iz = 0.0106;     % Moment of inertia around z axis
Xu = 0.2;       % Surge damping
Yv = 0.25;      % Sway damping
Nr = 0.005;    % Yaw damping


%% Identification

% Initial parameter guess
theta0 = [Iz, Xu, Yv, Nr]; 
% Add some noise to identification data
%dataId.nu = dataId.nu + 0.1*(0.5-rand(size(dataId.nu)));

[thetamin, objmin] = param_id(dataAId, theta0);

disp('Estimated parameters theta:');
disp(thetamin)
disp('Minimum of objective:');
disp(objmin)