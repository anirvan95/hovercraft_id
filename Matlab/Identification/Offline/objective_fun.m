function S = objective_fun(theta, data)
% OBJFUN objective function of parameter estimation 
% S = sum of squared differences between model and identification data
%
% Inputs:
%   theta : Vector with parameters to estimate
%   data  : Structure with identification data containing the following
%               dataId.name : name of data
%               data.t      : time of samples
%               dataId.nu   : state (u,v,r) 
%               dataId.U    : input (u1, u2)
%               dataId.h    : sampling time
%
% Output:
%   S : Value of objective function for current parameters theta

% assign parameters
m = 0.1763;
l = 0.0425;

Iz = theta(1);
Xu = theta(2);
Yv = theta(3);
Nr = theta(4);


% simulation setup
h = data.h;           % simulation time step
tmax = data.t(end);     % simulation time   
t = 0:h:tmax;         % simulation time 
nu0 = data.nu(:,1);       % initial condition

% upsampling input signal for simulation with nearest interpolation
Ures = interp1(data.t, data.U', t, 'previous', 'extrap');
data.U = Ures';

% wrap dynamicsId function into function that can be used by RK4
dynamics =@(nu, U) dynamics_reduced(nu, U, [m, l, Iz, Xu, Yv, Nr]);

% simulate system with current parameter using Euler forward
sim.f_discrete = @(nu,U) RK4(nu, U, h, dynamics);
sim.nu = nu0;
for k = 1:length(t) - 1
    sim.nu(:,k+1) = sim.f_discrete(sim.nu(:,k), data.U(:,k));
end

% compute predicted states at each (measured) sampling time by
% interpolating the simulation results at the measured sampling times
nuPred = interp1(t, sim.nu', data.t, 'nearest', 'extrap');


% compute sum of squared difference between model and identification data
% weight components of nu with standard 1/ std
weight = 1./std(data.nu, [], 2);
S = sum(sum( weight.*(nuPred' - data.nu).^2 ))/length(data.t);
%S = sum( sum( (nuPred(:,3)' - data.nu(3,:)).^2 ))/length(data.t_nu);
%S = sum(vecnorm(nuPred' - data.nu, 2, 2))/length(data.t_nu);

end