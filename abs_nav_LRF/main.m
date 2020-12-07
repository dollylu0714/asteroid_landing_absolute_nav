clear
clc

T =200; % 仿真时间
deltaT = 2; % 采样周期
len = fix(T/deltaT); % 仿真步数
tspan = 0:deltaT:T;

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
du_p = 1e-12; du_v = 1;
Qk = diag([du_p; du_p; du_p; du_v; du_v; du_v])^2;
Pk = diag(In_error.^2);
sta(1,:) = sqrt(diag(Pk));
% tspan2 = 0:0.2:deltaT;
for k = 1:len
    %     options = odeset('RelTol',1e-12,'AbsTol',1e-12);
    %     [tf,Xk11] = ode45(@dynamics,[0,deltaT],[Xe(k,:)]',options);
    %     [tf,Xk11] = ode45(@dynamics,tspan2,[Xe(k,:)]',options);
    %     Xk1=Xk11(end,1:6); % 用动力学递推的下一时刻的状态
    fai = JacobianF(Xe(k,:)); % 计算状态雅可比矩阵
    phi = eye(6)+fai*deltaT;
    Xk1 = (phi*Xe(k,:)')';
    
    du_LRF = 0.2; % m
    [Z_real] = abs_LRF(X(k+1,:),du_LRF); %模拟真实观测  X 探测器的真实状态, Xp 特征的状态
    Rk  = abs_LRF_cov(du_LRF);% 计算观测协方差;
    
    Hk = JacobianH_LRF(Xk1); % 计算观测雅可比矩阵
    Z_est = abs_LRF_est(Xk1); % 计算预测观测量
    
    [Xk_ekf, Pk_ekf] = ekf(phi, Qk, Xk1, Pk, Hk, Rk, Z_real, Z_est); % ekf 滤波函数
    %     I = eye(6);
    %     Pk1 = phi*Pk*phi' + Qk;
    %     Kk = Pk1*Hk'*(Hk*Pk1*Hk'+Rk)^-1;
    %     delta_Xk = Kk*(Z_real-Z_est);
    %     Xk_ekf = Xk1 + delta_Xk';
    %     Pk_ekf = (I-Kk*Hk)*Pk1*(I-Kk*Hk)' + Kk*Rk*Kk';
    
    Xe(k+1,:) = Xk_ekf;
    Pk = Pk_ekf;
    sta(k+1,:) = sqrt(diag(Pk)); % 标准差
    
%     Xe_dynamics(k+1,:) = Xk1;
%     Xe_est(k+1,:) = delta_Xk';
    %     innovation = Z_real-Z_est;
    %     inno_mean(k,1) = mean(innovation);
    
end
error = X - Xe;