function [x_next] = RK4(X,U,h,param)
% RK4 Runge-Kutta 4 integration
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future

k1 = dynamics(X         , U,param);
k2 = dynamics(X + h/2*k1, U,param); 
k3 = dynamics(X + h/2*k2, U,param); 
k4 = dynamics(X + h * k3, U,param); 

x_next = X + h*(k1/6 + k2/3 + k3/3 + k4/6);

end
