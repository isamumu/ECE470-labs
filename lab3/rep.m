function [ tau ] = rep( q,myrobot,obs)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

H1 = myrobot.links(1).A(q(1));myrobot.links(1);%disp(H1);
H2 = myrobot.links(2).A(q(2));myrobot.links(2);%disp(H2);
H3 = myrobot.links(3).A(q(3));%disp(H3);
H4 = myrobot.links(4).A(q(4));%disp(H4);
H5 = myrobot.links(5).A(q(5));%disp(H5);
H6 = myrobot.links(6).A(q(6));%disp(H6);

H1c = H1;                 [H01c,O1c]=tr2rt(H1c);
H2c = H1*H2;              [H02c,O2c]=tr2rt(H2c);
H3c = H1*H2*H3;           [H03c,O3c]=tr2rt(H3c); 
H4c = H1*H2*H3*H4;        [H04c,O4c]=tr2rt(H4c);
H5c = H1*H2*H3*H4*H5;     [H05c,O5c]=tr2rt(H5c);
H6c = H1*H2*H3*H4*H5*H6;  [H06c,O6c]=tr2rt(H6c);

%find rho i and rho i gradient acorrding to obs type
if strcmp(obs.type,'sph')
    [rho1, rho_g1] = rho_sphere( obs.R, obs.c, O1c);
    [rho2, rho_g2] = rho_sphere( obs.R, obs.c, O2c);
    [rho3, rho_g3] = rho_sphere( obs.R, obs.c, O3c);
    [rho4, rho_g4] = rho_sphere( obs.R, obs.c, O4c);
    [rho5, rho_g5] = rho_sphere( obs.R, obs.c, O5c);
    [rho6, rho_g6] = rho_sphere( obs.R, obs.c, O6c);
else
    disp('cyl')
    [rho1, rho_g1] = rho_cylinder( obs.R, obs.c, O1c);
    [rho2, rho_g2] = rho_cylinder( obs.R, obs.c, O2c);
    [rho3, rho_g3] = rho_cylinder( obs.R, obs.c, O3c);
    [rho4, rho_g4] = rho_cylinder( obs.R, obs.c, O4c);
    [rho5, rho_g5] = rho_cylinder( obs.R, obs.c, O5c);
    [rho6, rho_g6] = rho_cylinder( obs.R, obs.c, O6c);
end

n = 10^6;
%find F repulsive. If rho i > rho 0, F repulsive = 0
if rho1 > obs.rho0  
    disp(rho1); disp(obs.rho0);
    Frep1 = [0;0;0];
else
    Frep1 = (1/rho1 - 1/(obs.rho0))* (1/(rho1^2)) * rho_g1 * n; 
end

if rho2 > obs.rho0  
    disp(rho2);
    Frep2 = [0;0;0];
else
    Frep2 = (1/rho2 - 1/(obs.rho0))* (1/(rho2^2)) * rho_g2 * n; 
end

if rho3 > obs.rho0 
    disp(rho3);
    Frep3 = [0;0;0];
else
    Frep3 = (1/rho3 - 1/(obs.rho0))* (1/(rho3^2)) * rho_g3 * n; 
end

if rho4 > obs.rho0 
    disp(rho4);
    Frep4 = [0;0;0];
else
    Frep4 = (1/rho4 - 1/(obs.rho0))* (1/(rho4^2)) * rho_g4 * n;
end

if rho5 > obs.rho0   
    disp(rho5);
    Frep5 = [0;0;0];
else
    Frep5 = (1/rho5 - 1/(obs.rho0))* (1/(rho5^2)) * rho_g5 * n;
end

if rho6 > obs.rho0
    disp(rho6);
    Frep6 = [0;0;0];
else
    Frep6 = (1/rho6 - 1/(obs.rho0))* (1/(rho6^2)) * rho_g6 * n;
end
Frep = [Frep1 Frep2 Frep3 Frep4 Frep5 Frep6]

%Find Jacobian
Jv11 = cross([0; 0; 1;], O1c);

Jv21 = cross([0; 0; 1;], O2c);
Jv22 = cross(H01c(1:3, 3), (O2c-O1c));

Jv31 = cross([0; 0; 1;], O3c);
Jv32 = cross(H01c(1:3, 3), (O3c-O1c));
Jv33 = cross(H02c(1:3, 3), (O3c-O2c));

Jv41 = cross([0; 0; 1;], O4c);
Jv42 = cross(H01c(1:3, 3), (O4c-O1c));
Jv43 = cross(H02c(1:3, 3), (O4c-O2c));
Jv44 = cross(H03c(1:3, 3), (O4c-O3c));

Jv51 = cross([0; 0; 1;], O5c);
Jv52 = cross(H01c(1:3, 3), (O5c-O1c));
Jv53 = cross(H02c(1:3, 3), (O5c-O2c));
Jv54 = cross(H03c(1:3, 3), (O5c-O3c));
Jv55 = cross(H04c(1:3, 3), (O5c-O4c));

Jv61 = cross([0; 0; 1;], O6c);
Jv62 = cross(H01c(1:3, 3), (O6c-O1c));
Jv63 = cross(H02c(1:3, 3), (O6c-O2c));
Jv64 = cross(H03c(1:3, 3), (O6c-O3c));
Jv65 = cross(H04c(1:3, 3), (O6c-O4c));
Jv66 = cross(H05c(1:3, 3), (O6c-O5c));

z3 = zeros(3,1);
Jo1 = [Jv11 z3 z3 z3 z3 z3];
Jo2 = [Jv21 Jv22 z3 z3 z3 z3];
Jo3 = [Jv31 Jv32 Jv33 z3 z3 z3];
Jo4 = [Jv41 Jv42 Jv43 Jv44 z3 z3];
Jo5 = [Jv51 Jv52 Jv53 Jv54 Jv55 z3];
Jo6 = [Jv61 Jv62 Jv63 Jv64 Jv65 Jv66];

%retreive final tau for repulsive force
tau = ((Jo1)'*Frep1) + ((Jo2)'*Frep2) + ((Jo3)'*Frep3) + ((Jo4)'*Frep4) + ((Jo5)'*Frep5) + ((Jo6)'*Frep6);

%prevent division by 0 error
if norm(tau) == 0
    tau = tau';
else
    tau = tau/norm(tau);
    tau = tau';
end

end
