function [ rho_sphere,  rho_sphere_gradient] = rho_sphere( R, c, O)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
Ox = O(1);
Oy = O(2);
Oz = O(3);

Cx = c(1);
Cy = c(2);
Cz = c(3);

%denominator of b
b_den = sqrt((Ox-Cx)^2 + (Oy-Cy)^2 +(Oz-Cz)^2);

b = [(Ox-Cx)/b_den*R + Cx; (Oy-Cy)/b_den*R + Cy; (Oz-Cz)/b_den*R + Cz];
norm = b_den - R;

rho_sphere = norm;
rho_sphere_gradient = (O-b)/norm;

end
