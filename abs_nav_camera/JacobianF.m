function F = JacobianF(X) % 系统状态雅可比函数
global Omega;


x = X(1);
y = X(2);
z = X(3);
vx = X(4);
vy = X(5);
vz = X(6);
miu = 1e+14*3.9860044;
r = sqrt(x^2+y^2+z^2);

    F = zeros(6,6);     
    F(1,4) = 1;        
    F(2,5) = 1;     
    F(3,6) = 1;    
    F(4,1) = Omega^2;  F(4,5) = 2*Omega;  
    F(5,2) = Omega^2;    F(5,4) = -2*Omega;
   
   