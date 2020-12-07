function Xr = dynamics(t,X) %��ʵ��ϵͳ״̬�� X ����
dax = 0; day= 0;

rx = X(1);
ry = X(2);
rz = X(3);
vx = X(4);
vy = X(5);
vz = X(6);

global Omega;
global C0 C1 C2;
aimu = C0+C1.*t+C2.*(t^2);

% gravitysphericalharmonic�Ǽ�����г�������ĺ���
% [gx, gy, gz] = gravitysphericalharmonic( [rx+16000,ry+16000,rz+16000], 'Custom', 4,{'nsharco_near15.mat',@load}, 'Error' );


%% �˶�΢�ַ���
%  Xr=zeros(6,1);
drx=vx ;
dry=vy ;
drz=vz ;
% dvx=2*Omega*vy+Omega^2*rx+gx + aimu(1);
% dvy=-2*Omega*vx+Omega^2*ry+gy + aimu(2);
% dvz=gz+ aimu(3) ;
% ��������
dvx=2*Omega*vy+Omega^2*rx + aimu(1);
dvy=-2*Omega*vx+Omega^2*ry+ aimu(2);
dvz= aimu(3) ;
Xr = [drx;dry;drz;dvx;dvy;dvz];