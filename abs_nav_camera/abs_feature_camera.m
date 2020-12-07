function [Z_real,feature_a,feature_num_in_FOV] = abs_feature_camera( X_orbit,dp)

global feature feature_num feature_fixed
global camera_para

f = camera_para.f;
FOV = camera_para.FOV;

%% 可观测到的特征
% dA = (X_orbit(3))*tan(FOV/2);
% kkka = 0;
% image_coner = [X_orbit(1)-dA,X_orbit(2)+dA;X_orbit(1)-dA,X_orbit(2)-dA;X_orbit(1)+dA,X_orbit(2)+dA;X_orbit(1)+dA,X_orbit(2)-dA;];
% feature_a =zeros(feature_num+1,3);
% for kkkk=1:feature_num+1
%     if feature(kkkk,1)>X_orbit(1)-dA && feature(kkkk,1)<X_orbit(1)+dA && feature(kkkk,2)<X_orbit(2)+dA && feature(kkkk,2)>X_orbit(2)-dA
%         kkka=kkka+1;
%         feature_a(kkka,:) = feature(kkkk,:);
%     end
% end
% feature_num_in_FOV = kkka;

%% fixed feature 
feature_num_in_FOV = size(feature_fixed,1);
feature_a = feature_fixed;
%%
Z_real = zeros(feature_num_in_FOV*2,1);
for i = 1:feature_num_in_FOV
    a1 = X_orbit(1)-feature_a(i,1);   b1 = X_orbit(2) - feature_a(i,2) ; c1 = X_orbit(3) - feature_a(i,3);
    Z_real((i-1)*2+1,1) = f*a1/c1 + dp*rand(1,1);
    Z_real((i-1)*2+2,1) = f*b1/c1 + dp*rand(1,1);
end


end
