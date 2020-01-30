clc
clear all
close all

posdata2 = readmatrix('_slash_hoverOpti_slash_pose2d.csv');
commanded = readmatrix('_slash_cmd_thu.csv');


load('input_data_id_spec2_n.mat')
nu = [0;0;0];
eta = [];
for i = 1:(size(posdata2,1)-1)
    R = [cos(posdata2(i,4)),-sin(posdata2(i,4)),0;sin(posdata2(i,4)),cos(posdata2(i,4)),0;0,0,1];
    nu = [nu,inv(R)*(posdata2(i,2:4)-posdata2(i+1,2:4))'/dataId.t(1,2)];
    eta = [eta,posdata2(i,2:4)'];
end
eta = [eta,posdata2(i+1,2:4)'];
for k = 1:(length(dataId.t)-1)
    U(:,k) = generateU(dataId.u(k),dataId.r(k));
end
dataAId.name = 'identification data spec 2';
dataAId.t = dataId.t(1,1:(end-1));
dataAId.nu = nu;
dataAId.eta = eta;
dataAId.U = U;
dataAId.h = dataId.h;
save('spec2_2_data_id.mat', 'dataAId')