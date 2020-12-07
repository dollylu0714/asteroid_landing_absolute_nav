function [Xk, Pk] = ukf(xk,P,Y,Qk,Rk,delta_t,G,f,map_obs)
%%%%%%%% SVD-UKF %%%%%%%%
% Inputs:   xk: 状态估计值
%           P:  状态协方差矩阵
%           Y:  当前时刻测量值
%           Qk: 过程噪声
%           Rk: 观测噪声
%           delta_t: 时间间隔
%           G:  过程矩阵
% Output:   Xk: 当前状态估计值
%            Pk:  当前状态协方差矩阵

%% 权重系数
n=length(xk); m=length(Y);
alpha=0.5;                            % 一般取值范围 e-4=<alpha<1
beta=2;                               % Gaussian distribution: beta=2; 状态变量单变量：beta=0
kappa_k=0;                            % kappa_k=0 or =3-n
Lam=alpha^2*(n+kappa_k)-n;
weight_Wm=zeros(1,2*n+1);weight_Wc=zeros(1,2*n+1);
weight_Wm(1)=Lam/(n+Lam);weight_Wc(1)=Lam/(n+Lam)+(1-alpha^2+beta);
for i=1:2*n
    weight_Wm(i+1)=1/(2*(n+Lam));  %离散状态和观测的权重系数
    weight_Wc(i+1)=1/(2*(n+Lam));  %离散协方差的权重系数
end

%% 状态变量的sigma points  
X_chi=zeros(2*n+1,n);
X_chi(1,:)=xk';

co = sqrt(n+Lam);[U D V] = svd(P);D = sqrt(D);sigmasqr = U*D;
cholP0= co*sigmasqr;
for i=1:2*n
    if i>n
        X_chi(i+1,:)=xk-cholP0(:,i-n);
    else
        X_chi(i+1,:)=xk+cholP0(:,i);
    end
end

XX_=zeros(2*n+1,n);YY_=zeros(2*n+1,m);

for i=1:2*n+1
    XX_(i,:)=statestimate(delta_t,X_chi(i,:)')'; %状态变量的sigma points  
end

%% state & state covariance forcast
Xk_=zeros(n,1);
for i=1:2*n+1
    % Testone = weight_Wm(i)*XX_(i,:)';
    Xk_=Xk_+ weight_Wm(i)*XX_(i,:)'; % forcast of state variables
end
Pk_=G*Qk*G';% zeros(n) 协方差
for i=1:2*n+1
    Pk_=Pk_+weight_Wc(i)*((XX_(i,:)'-Xk_)*(XX_(i,:)'-Xk_)');
end

%% 观测变量的sigma points 
% YY_ 观测变量 的sigma点
% Y_ 
[Uk_ Dk_ Vk_] = svd(Pk_);
Dk_ = sqrt(Dk_);
% D = max(D, 0.05*eye(n));
sigmasqr = Uk_*Dk_;
cholPk_= co*sigmasqr;

X_chik_=zeros(2*n+1,n);
X_chik_(1,:)=Xk_';
for i=1:2*n
    if i>n
        X_chik_(i+1,:)=Xk_-cholPk_(:,i-n);
    else
        X_chik_(i+1,:)=Xk_+cholPk_(:,i);
    end
end
for i=1:2*n+1
    YY_(i,:)=Pointmeasurement(X_chik_(i,:)',map_obs,f);
end
%% 观测变量estimation
Y_=zeros(m,1);
for i=1:2*n+1
    Y_=Y_ + weight_Wm(i)*YY_(i,:)';  % propagated_y(i,:)= measurement(forcast_XX(i,:)',Xn(i,:),external_paras);
end
%% Coveriance update
Pyy = Rk;
Pxy = zeros(n,m);
for i=1:2*n+1
    Pyy = Pyy + weight_Wc(i)*((YY_(i,:)'-Y_)*(YY_(i,:)'-Y_)');
    Pxy = Pxy + weight_Wc(i)*(XX_(i,:)'-Xk_)*(YY_(i,:)'-Y_)';
end
%% coefficients kalman gain
K_k=Pxy/Pyy;
%%  Update
Xk=Xk_+K_k*(Y-Y_);
Pk=Pk_-K_k*Pyy*K_k';
end