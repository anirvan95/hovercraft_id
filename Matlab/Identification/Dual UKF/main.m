clc
clear all
close all

%following the aritificial data generation, 
%assuming full state observation
%TODO: have to add upsampling and downsampling 
load('spec2_2_data_id.mat')
h = dataAId.h;
t = dataAId.t;
%%Identification input
% u_fwd = 0.5;
% u_diff = 0.2 * idinput(length(t), 'prbs', [0, 10*h]); %10*h band, signal is constant over the interval 1/(10*h)
% U = u_fwd + [u_diff, - u_diff]';


pre_computed_params = [0.1763,0.0425]; %[m,l]
%states: x y phi u v r 
%observing x y phi u v r [check assumption]

x = zeros(6, 1);
%true hovercraft parameters
% params = [0.00013; 0.05; 0.048; 0.0001; 0.08;];
% The system model for both UKF's
xsbar = x; % initial belief of state
% Initial parameter guess
xpbar = [0.0106; 0.02; 0.02; 0.002]; % initial belief of parameter

Ps = 0.01*diag([0.05, 0.05, 0.08, 0.01, 0.01, 0.01]); %initial belief uncertainity
Pp = 10*diag([0.01 0.1 0.1 0.015]);
Rs = 100*diag([1.2^2 1.2^2 0.45^2 0.5^2 0.5^2 0.1^2]); %process noise
Rp = 0.001*diag([0.01^2 0.05^2 0.05^2 0.05^2]);

Q = 0.1*diag([0.02^2 0.02^2 0.025^2 0.01^2 0.01^2 0.01^2]); %measurement noise
Hs = eye(6); %full state observation using motion capture
gamma = 0.995;
Xsbar(:,1) = xsbar;
Xpbar(:,1) = xpbar;
X(:,1) = x;
Ppp=[Pp(1,1);Pp(2,2)];

disp('Estimating online parameters from hovercraft model')

start_index = 500;
for loop = start_index:length(t)-1
    loop
    u = dataAId.U(:,loop);
%   x = RK4(x,u,h,params);
    
    %x = dynamic_step(x,u,param,orig_params,dt,0.01);
    z = [dataAId.eta(:,loop);dataAId.nu(:,loop)];
    xsbar = z;
    %% Prediction
    %%Parameter
    %UKF defining tuning variables and weights
    L_p=numel(xpbar);
    alpha_p=1e-2; %tune here.. for parameters
    ki_p=2;
    beta_p=2;
    lambda_p=alpha_p^2*(L_p+ki_p)-L_p;
    c_p=L_p+lambda_p;
    Wm_p=[lambda_p/c_p 0.5/c_p+zeros(1,2*L_p)];           %weights for means
    Wc_p=Wm_p;
    Wc_p(1)=Wc_p(1)+(1-alpha_p^2+beta_p);               %weights for covariance
    c_p=sqrt(c_p);
    
    %UKF defining sigma points Xt-1, Step 2 from book
    Pp = (Pp + Pp')/2;
    Xp=sigmas(xpbar,Pp,c_p);                            %sigma points around x
    
    %parameter prediction using unscented transform
    xphat = zeros(L_p,1);
    for l=1:size(Xp,2)
        Yp(:,l)=Xp(:,l); %The map of parameter is an identity
        xphat=xphat+Wm_p(l)*Yp(:,l);
    end
    Y1p=Yp-xphat(:,ones(1,size(Xp,2)));
    Pphat=Y1p*diag(Wc_p)*Y1p'+Rp;
    
    %%Taking sigma points around the predicted state %State 6: from book
    Pphat = (Pphat + Pphat')/2;
    Xphat=sigmas(xphat,Pphat,c_p);


    %%State
    %UKF defining tuning variables and weights
    L_s=numel(xsbar);
    alpha_s=1e-3;
    ki_s=0;
    beta_s=2;
    lambda_s=alpha_s^2*(L_s+ki_s)-L_s;
    c_s=L_s+lambda_s;
    Wm_s=[lambda_s/c_s 0.5/c_s+zeros(1,2*L_s)];           %weights for means
    Wc_s=Wm_s;
    Wc_s(1)=Wc_s(1)+(1-alpha_s^2+beta_s);               %weights for covariance
    c_s=sqrt(c_s);
    
    %UKF defining sigma points Xt-1
    Ps = (Ps + Ps')/2;
    Xs=sigmas(xsbar,Ps,c_s);                            %sigma points around x
    xsbar_prev = xsbar;
    %state prediction using unscented transform
    xshat = zeros(L_s,1);
    for l=1:size(Xs,2)%check if it is 2N or not
        Ys(:,l)=RK4(Xs(:,l),u,h,xphat); %The map of state
        xshat=xshat+Wm_s(l)*Ys(:,l);
    end
    Y1s=Ys-xshat(:,ones(1,size(Xs,2)));
    Pshat=Y1s*diag(Wc_s)*Y1s'+Rs;
    
    %%Taking sigma points around the predicted state %State 6: from book
    Pshat = (Pshat + Pshat')/2; %%add diagnal terms - avoid singular values
    Xshat=sigmas(xshat,Pshat,c_s);

    
    %% Correction
    %%UKF on measurement for state
    
    %measurement prediction using unscented transform
    %parameter prediction using unscented transform, Step 3,4,5: from book
    zshat = zeros(L_s,1);
    for l=1:size(Xs,2)%check if it is 2N or not
        Yz(:,l)=Xshat(:,l); %The map of state
        zshat=zshat+Wm_s(l)*Yz(:,l);
    end
    
    Y1z=Yz-zshat(:,ones(1,size(Yz,2)));
    Pzhat=Y1z*diag(Wc_s)*Y1z'+Q;
    %%Evaluating cross covariance. ilne 10 from book
    Psz=Y1s*diag(Wc_s)*Y1z';
       
    %%State correction
     
    Ks=Psz*inv(Pzhat);
    xsbar=xshat+Ks*(z-zshat);                              %state update
    Ps=Pshat-Ks*Pzhat*Ks';                                %covariance update

    
    %Hope it is correct.. !!
    
    %%UKF on measurement for parameters
    zphat = zeros(L_s,1);
    for l=1:size(Xp,2)%check if it is 2N or not
        Yzp(:,l) = RK4(xsbar_prev,u,h,Xp(:,l)); %Notice that I am generating expected measurement for each sigma point of parameter distribution
        zphat=zphat+Wm_p(l)*Yzp(:,l);
    end
    Y1zp=Yzp-zphat(:,ones(1,size(Xp,2)));
    Pzhat_2=Y1zp*diag(Wc_p)*Y1zp'+Q;
    Ppz = Y1p*diag(Wc_p)*Y1zp';  %critical point, generating cross covariance
    
    Kp=Ppz*inv(Pzhat_2);
    xpbar=xphat+Kp*(z-zphat);                              %state update
    Pp=Pphat-Kp*Pzhat_2*Kp';                            %covariance update

    %disp(Kp*Ppz')
    Rp = Rp*gamma;
    Xsbar(:,loop+1) = xsbar;
    Xpbar(:,loop+1) = xpbar;
    X(:,loop+1) = z;
    
%     plot(z(1), z(2), '.b')
%     hold on
%     plot(xsbar(1), xsbar(2), '.r')
%     grid on
%     drawnow
%     NN(:,loop) = norm(params-xpbar);
    Ppp(:,loop+1) = [Pp(1,1);Pp(2,2)];
end


figure,
subplot(2,2,1),plot(t(start_index:end),Xpbar(1,start_index:end),'r','Linewidth',2')
ylabel("I_z",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
grid on
subplot(2,2,2),plot(t(start_index:end),Xpbar(2,start_index:end),'g','Linewidth',2')
ylabel("X_u",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
grid on
subplot(2,2,3),plot(t(start_index:end),Xpbar(3,start_index:end),'b','Linewidth',2')
ylabel("Y_v",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
grid on
subplot(2,2,4),plot(t(start_index:end),Xpbar(4,start_index:end),'k','Linewidth',2')
ylabel("N_r",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
grid on

disp('Done')
figure,
plot(X(1,:), X(2,:), '-b','Linewidth',2)
hold on
plot(Xsbar(1,:), Xsbar(2,:), '.r')
grid on