function Rk  = abs_feature_cov(feature,dp,feature_num_in_FOV)
% ������۲����Э����
Rk = zeros(feature_num_in_FOV*2,feature_num_in_FOV*2);
Rk = dp^2.*eye(feature_num_in_FOV*2,feature_num_in_FOV*2) ;

end
