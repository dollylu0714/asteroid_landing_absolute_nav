function Z_est = abs_fusion_est(X,feature,feature_num_in_FOV)
global camera_para

f = camera_para.f;
FOV = camera_para.FOV;

%%
Z_est = zeros(feature_num_in_FOV*2+1,1);
for i = 1:feature_num_in_FOV
    a1 = X(1)-feature(i,1);   b1 = X(2) - feature(i,2) ; c1 = X(3) - feature(i,3);
    Z_est((i-1)*2+1,1) = f*a1/c1;
    Z_est((i-1)*2+2,1) = f*b1/c1;
end
Z_est(feature_num_in_FOV*2+1,1) = X(3);
end