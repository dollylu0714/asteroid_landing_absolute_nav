function [Z_real] = abs_LRF( X_orbit,dp)

Z_real = X_orbit(3)+dp*rand(1,1);

end
