function [ rho_cylinder, rho_cylinder_gradient ] = rho_cylinder( R, c, O )
%RHO_CYLINER Summary of this function goes here
%   Detailed explanation goes here
Ox = O(1);
Oy = O(2);
Oz = O(3);

Cx = c(1);
Cy = c(2);
Cz = Oz;

%denominator of b
b_den = sqrt((Ox-Cx)^2 + (Oy-Cy)^2);

b = [R*(Ox-Cx)/b_den + Cx; R*(Oy-Cy)/b_den + Cy; Cz];
norm = b_den - R;

rho_cylinder = norm;
rho_cylinder_gradient = (O-b)/norm;

end

