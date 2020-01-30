function dnudt = dynamics_reduced(nu, U, param)
% DYNAMICSID calculates the function f(X,U,theta) of the dynamic system
% dX/dt = f(t,X,theta) for the parameters theta
%
% Inputs:
%   X1 = x, X2 = y, X3 = psi, X4 = u, X5 = v, X6 = r
%
% Output:
%   dnudt : Derivative of state (velocity) vector

% parameters of hovercraft
m = param(1);      % mass of hovercraft
l = param(2);
Iz = param(3);     % moment of inertia around z axis
Xu = param(4);     % surge damping
Yv = param(5);     % sway damping
Nr = param(6);     % yaw damping

dnudt = zeros(3, 1);

M_inv = diag([1/m, 1/m, 1/Iz]);

% coriolis matrix
C = [0, 0, - m * nu(2);
     0, 0, m * nu(1);
     m * nu(2), - m * nu(1), 0];

% damping matrix
D = diag([Xu, Yv, Nr]);

%input matrix
B = [1, 1, -1, -1;
     0, 0, 0, 0;
     l, -l, -l, l];

% Decrease computational time by writing all matrix multiplication explicitly
dnudt = M_inv * (B * U - C * nu - D * nu);

end