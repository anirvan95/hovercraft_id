%%Open loop simulation of modified TinyWhoover 
clc
clear all
close all

load('spec3_3_data_id.mat')

%% simulation settings
t = dataAId.t;       % time vector
h = 0.001;

%% TinyWhoover parameters
m = 0.1763;
l = 0.0425;
Iz = 0.01;      % moment of inertia around z axis
Xu = 0.098191;     % linear damping
Yv = 0.0567;
Nr = 0.0071;     % yaw damping

param = [m, l, Iz, Xu, Yv, Nr];

% initial state vector
%X1 = x, X2 = y, X3 = psi, X4 = u, X5 = v, X6 = r

X0 = zeros(6, 1);
X0(1:3) = dataAId.eta(:,1);

rk4.name = 'RK4';
% RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
rk4.f_discrete = @(X,U) RK4(X, U, h, @(X,U) dynamics(X,U,param));
rk4.X = X0;
for k = 1:length(t) - 1
    rk4.U(:, k) = dataAId.U(:,k);
    rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), rk4.U(:, k));
end
visualize(t, rk4.X, 2)

figure('DefaultAxesFontSize', 16)
hold on
plot(rk4.X(1, :), rk4.X(2, :), 'b', 'LineWidth', 0.75)
plot(rk4.X(1,1), rk4.X(2,1), 'ob', 'MarkerFaceColor','b','MarkerSize', 10)
set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
grid
axis equal
xlabel('x [m]')
ylabel('y [m]')
