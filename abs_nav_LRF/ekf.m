function [Xk, Pk] = ekf(fai, Qk, Xk1, Pk, Hk, Rk, Z_real, Z_est) % ekf ÂË²¨º¯Êý

%%%%% input%%%%
% fai  = Jacobian matrix of state equation
% Qk = system error covariance
% Xk =  transformed state
% Pk =  state error covariance
% Hk = Jacobian matrix of measurement equation
% Rk = measurement error covariance 
% Z_real = real measurement 
% Z_est =  estimated measurement
    
  I = eye(6);
  Pk1 = fai*Pk*fai' + Qk;
  Kk = Pk1*Hk'*(Hk*Pk1*Hk'+Rk)^-1;
  delta_Xk = Kk*(Z_real-Z_est);     
  Xk = Xk1 + delta_Xk';
  Pk = (I-Kk*Hk)*Pk1*(I-Kk*Hk)' + Kk*Rk*Kk';
end