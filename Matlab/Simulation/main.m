%%Open loop simulation of modified TinyWhoover 
clc
clear all
close all

%% simulation settings
h_arr = [0.005];          % sample time
for loop = 1:length(h_arr)
    h = h_arr(loop);
tmax = 10;          % simulation time   
t = 0:h:tmax;       % time vector

%% Hovercraft parameters
m = 0.1763;
l = 0.0425;
Iz = 0.01;      % moment of inertia around z axis
Xu = 0.2;     % linear damping
Xv = 0.16;
Nr = 0.05;     % yaw damping
u_max = 1.0;      % maximum input

param = [m, l, Iz, Xu, Xv, Nr];

% initial state vector
%X1 = x, X2 = y, X3 = psi, X4 = u, X5 = v, X6 = r

X0 = zeros(6, 1);
X0(1:2) = [2, 2];

u = 0.1;
r = 0.3*idinput(length(t), 'prbs', [0, 10*h]); %10*h band, signal is constant over the interval 1/(10*h)

Ut = [0;0;0;0];
rk4.name = 'RK4';
% RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
rk4.f_discrete = @(X,U) RK4(X, U, h, @(X,U) dynamics(X,U,param));
rk4.X = X0;
rk4.U = Ut;
for k = 1:length(t) - 1
    X_d = dynamics(rk4.X(:,k), Ut, param);
    Ut = generateU(u,r(k));
    rk4.U(:, k) = Ut;
    rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), Ut);
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
end