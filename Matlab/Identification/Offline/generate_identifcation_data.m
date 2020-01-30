%% Generate artificial identification tracking data
clc
clear all
close all

%% Load generated ID input
load input_data_id_spec1.mat

%% Simulation parameters 
h_sim = 0.005; %For RK4
h_out = dataId.h; %sampling time of data
t_max = max(dataId.t);
t_sim = 0:h_sim:t_max;
param = dataId.param;

%%State
X0 = zeros(6, 1);

sim.f_discrete = @(X,U) RK4(X, U, h_sim, @(X,U) dynamics(X,U,param));
sim.X = X0;

k = 1;
for k_sim = 1:length(t_sim) - 1
    if(t_sim(k_sim)<=dataId.t(k))
        k = k;
    else
        k = k+1;
    end
    U(:,k_sim) = generateU(dataId.u(k),dataId.r(k));
    sim.X(:,k_sim+1) = sim.f_discrete(sim.X(:,k_sim), U(:,k_sim));
end
U(:,k_sim+1) = generateU(dataId.u(k),dataId.r(k));
sim.U = U;

visualize(t_sim, sim.X, 10)
%% save artificial tracking data
add_noise = true;      % add noise to simulated velocity vector
noise_level = 0.4;      % amplitude of noise
nu = sim.X(4:6, :);
if add_noise
    nu = nu + noise_level * std(nu, [], 2).*randn(size(nu));
end

step = h_out/h_sim;
dataId.name = 'artificial tracking data';
dataId.true_param = dataId.param;
dataId.t = t_sim(1:step:end);
dataId.eta = sim.X(1:3, 1:step:end);
dataId.nu = nu(:, 1:step:end);
dataId.U = U(:,1:step:end);
dataId.h = h_sim;
save('tracking_data_spec_1.mat', 'dataId')
