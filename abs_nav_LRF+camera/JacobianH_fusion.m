function H = JacobianH_fusion(X,feature,feature_num_in_FOV) % 量测雅可比函数
global camera_para
f = camera_para.f;
FOV = camera_para.FOV;

H = zeros(feature_num_in_FOV*2+1,6);

for i = 1:feature_num_in_FOV
    a1 = X(1)-feature(i,1);   b1 = X(2) - feature(i,2) ; c1 = X(3) - feature(i,3);
    H((i-1)*2+1,1) = f/c1;  H((i-1)*2+1,3) = -f*a1/c1^2;
    H((i-1)*2+2,2) = f/c1;    H((i-1)*2+2,3) = -f*b1/c1^2;
end
H(feature_num_in_FOV*2+1,:) = [0,0,1,0,0,0];
end

