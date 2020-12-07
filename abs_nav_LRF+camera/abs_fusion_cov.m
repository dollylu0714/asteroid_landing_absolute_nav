function Rk  = abs_fusion_cov(feature,dp,feature_num_in_FOV)
% 特征点观测误差协方差
Rk = zeros(feature_num_in_FOV*2+1,feature_num_in_FOV*2+1);
Rk(1:feature_num_in_FOV*2,1:feature_num_in_FOV*2) = dp(1)^2.*eye(feature_num_in_FOV*2,feature_num_in_FOV*2) ;
Rk(feature_num_in_FOV*2+1,feature_num_in_FOV*2+1) = dp(2)^2;
end
