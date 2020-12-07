clear
clc

T =200; % 仿真时间
deltaT = 2; % 采样周期
len = fix(T/deltaT); % 仿真步数
tspan = 0:deltaT:T;
%%  光学参数
global camera_para
camera_para.FOV =20/180*pi;
camera_para.f = 0.717;
camera_para.size = 1048;
camera_para.error_pixel = 0.5;

f = camera_para.f;
FOV = camera_para.FOV;

pixel_size = camera_para.f * tan(camera_para.FOV/2) / (camera_para.size/2);
Kx = 1/(pixel_size);
Ky = 1/(pixel_size);

global feature feature_num feature_fixed
% 建立特征数据库
feature_num =200;
for i=1:feature_num+1
    feature(i,:) = [-100+200*rand(1,1),-100+200*rand(1,1),-2+4*rand(1,1)];
end
feature(feature_num+1,:) = [0,0,262.5];

feature_fixed=[0, 0, 0; 100, 100*sqrt(3), 0; 200, 0, 0];

%%  初始位置与标称轨道
% 初始状态
state_num = 6;
In_pos = [10, 10, 500]';
In_vel = [-0.2, 0.1, -0.5]';
In_state = [In_pos;In_vel];
In_error  = [30,30,50,0.1,0.1,0.1];
X(1,:) = [In_pos', In_vel']; % 真实的位置速度初值
Xe(1,:)=X(1,:)+In_error*rand(1,1); % 带误差的位置速度估计初值
TB =[0.99997,0,0; 0,0.877952,0.478686; 0,-0.478686,0.877952 ];   %  本体系到着陆系转移矩阵
TL = inv(TB)  ;        %着陆系到本体系转移矩阵

% 标称轨道
global Omega;
Omega = 3.314e-004;          %小天体自转角速率
global C0 C1 C2;
Ter_pos = [2 2 30]';
Ter_vel = [0 0 0]';
Ter_acc = [0 0 0]';
[ C0, C1, C2 ] = Apollogl( Ter_acc, Ter_vel, Ter_pos, In_vel, In_pos, T );
options = odeset('RelTol',1e-12,'AbsTol',1e-12);
[T0,true_state] = ode45(@dynamics,tspan,In_state,options);
X = true_state;

%%  EKF 滤波器
% du_p = 1e-12; du_v = 1;
% Qk = diag([du_p; du_p; du_p; du_v; du_v; du_v])^2;
% Pk = diag(In_error.^2);
% sta(1,:) = sqrt(diag(Pk));
% % tspan2 = 0:0.2:deltaT;
% for k = 1:len
%     %     options = odeset('RelTol',1e-12,'AbsTol',1e-12);
%     %     [tf,Xk11] = ode45(@dynamics,[0,deltaT],[Xe(k,:)]',options);
%     %     [tf,Xk11] = ode45(@dynamics,tspan2,[Xe(k,:)]',options);
%     %     Xk1=Xk11(end,1:6); % 用动力学递推的下一时刻的状态
%     fai = JacobianF(Xe(k,:)); % 计算状态雅可比矩阵
%     phi = eye(6)+fai*deltaT;
%     Xk1 = (phi*Xe(k,:)')';
%     
%     du_c_m = 0.003; % m
%     du_LRF = 0.2;% m
%     [Z_real,feature_a,n] = abs_fusion(X(k+1,:),[du_c_m,du_LRF]); %模拟真实观测  X 探测器的真实状态, Xp 特征的状态
%     Rk  = abs_fusion_cov(feature_a,[du_c_m,du_LRF],n);% 计算观测协方差;
%     
%     Hk = JacobianH_fusion(Xk1,feature_a,n); % 计算观测雅可比矩阵
%     Z_est = abs_fusion_est(Xk1,feature_a,n); % 计算预测观测量
%     
%     [Xk_ekf, Pk_ekf] = ekf(phi, Qk, Xk1, Pk, Hk, Rk, Z_real, Z_est); % ekf 滤波函数
%     %     I = eye(6);
%     %     Pk1 = phi*Pk*phi' + Qk;
%     %     Kk = Pk1*Hk'*(Hk*Pk1*Hk'+Rk)^-1;
%     %     delta_Xk = Kk*(Z_real-Z_est);
%     %     Xk_ekf = Xk1 + delta_Xk';
%     %     Pk_ekf = (I-Kk*Hk)*Pk1*(I-Kk*Hk)' + Kk*Rk*Kk';
%     
%     Xe(k+1,:) = Xk_ekf;
%     Pk = Pk_ekf;
%     sta(k+1,:) = sqrt(diag(Pk)); % 标准差
%     
% %     Xe_dynamics(k+1,:) = Xk1;
% %     Xe_est(k+1,:) = delta_Xk';
%     %     innovation = Z_real-Z_est;
%     %     inno_mean(k,1) = mean(innovation);
%     
% end
% error = X - Xe;
% error2 = X - Xe_dynamics;
%%  UKF 滤波器
du_p = 1e-5; du_v = 1;
Qk = diag([du_p; du_p; du_p; du_v; du_v; du_v])^2;
Pk = diag(In_error.^2);
sta(1,:) = sqrt(diag(Pk));
Xk_ukf = Xe(1,:) ;
tspan2 = 0:0.2:deltaT;
for kk = 1:len
    kk+1
    %     du_c_m = 0.003; % m
    %     [Z_real,feature_a,n] = abs_feature_camera(X(k+1,:),du_c_m);
    %     Rk  = abs_feature_cov(feature_a,du_c_m,n);% 计算观测协方差;
    %
    %     [Xk_ukf, Pk_ukf] = ukf(xk,Pk,Z_real,Qk,Rk, delta_t);
    %     Xe(k+1,:) = Xk_ukf;
    %     Pk = Pk_ukf;
    %     sta(k+1,:) = sqrt(diag(Pk)); % 标准差
    
    % 模拟观测量
    du_c_m = 0.003; % m
    du_LRF = 0.2;% m
    [Z_real,feature_a,n] = abs_fusion(X(kk+1,:),[du_c_m,du_LRF]);
    Rk  = abs_fusion_cov(feature_a,[du_c_m,du_LRF],n);% 计算观测协方差;
    
    % sigma点离散
    L=numel(Xk_ukf);                                 %numer of states
    m=numel(Z_real);                                 %numer of measurements
    alpha=1e-3;                                 %default, tunable
    ki=0;                                       %default, tunable
    beta=2;                                     %default, tunable
    lambda=alpha^2*(L+ki)-L;                    %scaling factor
    c=L+lambda;                                 %scaling factor
    Wm=[lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
    Wc=Wm;
    Wc(1)=Wc(1)+(1-alpha^2+beta);               %weights for covariance
    c=sqrt(c);
    X_sigma=sigmas(Xe(kk,:)',Pk,c);                            %sigma points around x
    
    % 状态的无损变换
    num=size(X_sigma,2);
    x1=zeros(L,1);
    X1=zeros(L,num);
    for k=1:num
        options = odeset('RelTol',1e-12,'AbsTol',1e-12);
        [tf,X11_sigma] = ode45(@dynamics,tspan2,[X_sigma(:,k)'],options);
        X1_sigma=X11_sigma(end,1:6); % 用动力学递推的下一时刻的状态
        X1(:,k)=X1_sigma';
        x1=x1+Wm(k)*X1(:,k);
    end
    X2=X1-x1(:,ones(1,num));
    P1=X2*diag(Wc)*X2'+Qk;
    
    % 观测的无损变换
    num2=size(X1,2);
    z1=zeros(m,1);
    Z1=zeros(m,num2);
    for k=1:num2
        %         Z1(:,k)=f(X(:,k));
        Z1(:,k) = abs_fusion_est(X1(:,k)',feature_a,n);
        z1=z1+Wm(k)*Z1(:,k);
    end
    Z2=Z1-z1(:,ones(1,num2));
    P2=Z2*diag(Wc)*Z2'+Rk;
    
    % 更新
    P12=X2*diag(Wc)*Z2';                        %transformed cross-covariance
    K=P12*inv(P2);
    Xk_ukf=x1+K*(Z_real-z1);                              %state update
    Pk_ukf=P1-K*P12';                             %covariance update
    
    % 整理结果
    Xe(kk+1,:) = Xk_ukf';
    Pk = Pk_ukf;
    sta(kk+1,:) = sqrt(diag(Pk)); % 标准差
    
    Xe_dynamics(kk+1,:) = x1';
    Xe_est(kk+1,:) = (K*(Z_real-z1))';
    innovation = Z_real-z1;
    inno_mean(kk,1) = mean(innovation);
    
end
error = X - Xe;
error2 = X - Xe_dynamics;

