clear
clc

T =200; % ����ʱ��
deltaT = 2; % ��������
len = fix(T/deltaT); % ���沽��
tspan = 0:deltaT:T;

%%  ��ʼλ�����ƹ��
% ��ʼ״̬
state_num = 6;
In_pos = [10, 10, 500]';
In_vel = [-0.2, 0.1, -0.5]';
In_state = [In_pos;In_vel];
In_error  = [30,30,50,0.1,0.1,0.1];
X(1,:) = [In_pos', In_vel']; % ��ʵ��λ���ٶȳ�ֵ
Xe(1,:)=X(1,:)+In_error*rand(1,1); % ������λ���ٶȹ��Ƴ�ֵ
TB =[0.99997,0,0; 0,0.877952,0.478686; 0,-0.478686,0.877952 ];   %  ����ϵ����½ϵת�ƾ���
TL = inv(TB)  ;        %��½ϵ������ϵת�ƾ���

% ��ƹ��
global Omega;
Omega = 3.314e-004;          %С������ת������
global C0 C1 C2;
Ter_pos = [2 2 30]';
Ter_vel = [0 0 0]';
Ter_acc = [0 0 0]';
[ C0, C1, C2 ] = Apollogl( Ter_acc, Ter_vel, Ter_pos, In_vel, In_pos, T );
options = odeset('RelTol',1e-12,'AbsTol',1e-12);
[T0,true_state] = ode45(@dynamics,tspan,In_state,options);
X = true_state;
%%  EKF �˲���
du_p = 1e-12; du_v = 1;
Qk = diag([du_p; du_p; du_p; du_v; du_v; du_v])^2;
Pk = diag(In_error.^2);
sta(1,:) = sqrt(diag(Pk));
% tspan2 = 0:0.2:deltaT;
for k = 1:len
    %     options = odeset('RelTol',1e-12,'AbsTol',1e-12);
    %     [tf,Xk11] = ode45(@dynamics,[0,deltaT],[Xe(k,:)]',options);
    %     [tf,Xk11] = ode45(@dynamics,tspan2,[Xe(k,:)]',options);
    %     Xk1=Xk11(end,1:6); % �ö���ѧ���Ƶ���һʱ�̵�״̬
    fai = JacobianF(Xe(k,:)); % ����״̬�ſɱȾ���
    phi = eye(6)+fai*deltaT;
    Xk1 = (phi*Xe(k,:)')';
    
    du_LRF = 0.2; % m
    [Z_real] = abs_LRF(X(k+1,:),du_LRF); %ģ����ʵ�۲�  X ̽��������ʵ״̬, Xp ������״̬
    Rk  = abs_LRF_cov(du_LRF);% ����۲�Э����;
    
    Hk = JacobianH_LRF(Xk1); % ����۲��ſɱȾ���
    Z_est = abs_LRF_est(Xk1); % ����Ԥ��۲���
    
    [Xk_ekf, Pk_ekf] = ekf(phi, Qk, Xk1, Pk, Hk, Rk, Z_real, Z_est); % ekf �˲�����
    %     I = eye(6);
    %     Pk1 = phi*Pk*phi' + Qk;
    %     Kk = Pk1*Hk'*(Hk*Pk1*Hk'+Rk)^-1;
    %     delta_Xk = Kk*(Z_real-Z_est);
    %     Xk_ekf = Xk1 + delta_Xk';
    %     Pk_ekf = (I-Kk*Hk)*Pk1*(I-Kk*Hk)' + Kk*Rk*Kk';
    
    Xe(k+1,:) = Xk_ekf;
    Pk = Pk_ekf;
    sta(k+1,:) = sqrt(diag(Pk)); % ��׼��
    
%     Xe_dynamics(k+1,:) = Xk1;
%     Xe_est(k+1,:) = delta_Xk';
    %     innovation = Z_real-Z_est;
    %     inno_mean(k,1) = mean(innovation);
    
end
error = X - Xe;