function [tau] = att( q,qf,myrobot )
%UNTITLED2 Summary of this function goes here
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

%find Oi for every link
H1x = myrobot.links(1).A(qf(1));%disp(H1);
H2x = myrobot.links(2).A(qf(2));%disp(H2);
H3x = myrobot.links(3).A(qf(3));%disp(H3);
H4x = myrobot.links(4).A(qf(4));%disp(H4);
H5x = myrobot.links(5).A(qf(5));%disp(H5);
H6x = myrobot.links(6).A(qf(6));%disp(H6);

H1f = H1x;                    [H01f,O1f]=tr2rt(H1f);
H2f = H1x*H2x;                [H02f,O2f]=tr2rt(H2f);
H3f = H1x*H2x*H3x;             [H03f,O3f]=tr2rt(H3f);
H4f = H1x*H2x*H3x*H4x;          [H04f,O4f]=tr2rt(H4f);
H5f = H1x*H2x*H3x*H4x*H5x;       [H05f,O5f]=tr2rt(H5f);
H6f = H1x*H2x*H3x*H4x*H5x*H6x;    [H06f,O6f]=tr2rt(H6f);

%calculate F attractive
Fatt1 = (O1c - O1f)*(-1);
Fatt2 = (O2c - O2f)*(-1);
Fatt3 = (O3c - O3f)*(-1);
Fatt4 = (O4c - O4f)*(-1);
Fatt5 = (O5c - O5f)*(-1);
Fatt6 = (O6c - O6f)*(-1);
Fatt = [Fatt1 Fatt2 Fatt3 Fatt4 Fatt5 Fatt6];
Fatt;
myrobot.links(1).d ;
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

%retreive final tau for attractive force
tau = ((Jo1)'*Fatt1) + ((Jo2)'*Fatt2) + ((Jo3)'*Fatt3) + ((Jo4)'*Fatt4) + ((Jo5)'*Fatt5) + ((Jo6)'*Fatt6);

tau = tau/norm(tau);
tau = tau';
end