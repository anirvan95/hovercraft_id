function dXdt = dynamics_hov(X, U, param)
% DYNAMICS Calculate the function f(X,U) of the dynamic system
% dX/dt = f(X,U)
%
% Inputs:
%   X : State (column) vector, X1 = x, X2 = y, X3 = psi, X4 = u, X5 = v, X6 = r
%   U : Input (column) vector, U1 = Bl (back left thruster), U2 = Br (back
%   right thruster) U3 = Fl (front left thruster), U4 = Fr (front right thruster)
%   param : Parameters of hovercraft (row vector)
% Output:
%   dXdt : Derivative of state vector

eta = X(1:3);   % position and orientation in ground frame
nu = X(4:6);    % linear and angular velocity in body frame

% parameters of hovercraft
m = param(1);      % mass of hovercraft
l = param(2);
Iz = param(3);     % moment of inertia around z axis
Xu = param(4);     % surge damping
Yv = param(5);     % sway damping
Nr = param(6);     % yaw damping

% origin of body frame is assumed at center of gravity

% inverse of mass matrix
M_inv = diag([1/m, 1/m, 1/Iz]);

% coriolis matrix
C = [0, 0, - m * nu(2);
     0, 0, m * nu(1);
     m * nu(2), - m * nu(1), 0];

% damping matrix
D = diag([Xu, Yv, Nr]);

% input matrix
B = [1, 1, -1, -1;
     0, 0, 0, 0;
     l, -l, -l, l];

% rotation matrix from body to ground frame
R_z_psi = [cos(eta(3)), -sin(eta(3)), 0;
           sin(eta(3)),  cos(eta(3)), 0;
                0     ,       0     , 1];

dXdt = zeros(6, 1);

% temporal derivative of x, y, psi
dXdt(1:3) = R_z_psi * nu;
% temporal derivative of u, v, r
dXdt(4:6) = M_inv * (B * U - C * nu - D * nu);

end