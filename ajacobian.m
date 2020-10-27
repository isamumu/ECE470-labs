function [ aJ ] = ajacobian( q ,myrobot )
%Analytical jacobian solver, q is the joint variable and myrobot is the robot
% first solve for the geometric jacobian
J=zeros(6,6);
%first solve angular jacobian:
H1 = myrobot.links(1).A(q(1));%disp(H1);
H2 = myrobot.links(2).A(q(2));%disp(H2);
H3 = myrobot.links(3).A(q(3));%disp(H3);
H4 = myrobot.links(4).A(q(4));%disp(H4);
H5 = myrobot.links(5).A(q(5));%disp(H5);
H6 = myrobot.links(6).A(q(6));%disp(H6);
H00 = [0,0,1];O00 = [0,0,0];
H01 = H1;               [H01,O01]=tr2rt(H01);%disp(O01)
H02 = H1*H2;            [H02,O02]=tr2rt(H02);%disp(O02)%disp(H02);
H03 = H1*H2*H3;         [H03,O03]=tr2rt(H03);%disp(O03)%disp(H03);
H04 = H1*H2*H3*H4;      [H04,O04]=tr2rt(H04);%disp(O04)%disp(H04);
H05 = H1*H2*H3*H4*H5;   [H05,O05]=tr2rt(H05);%disp(O05)%disp(H05);
H06 = H1*H2*H3*H4*H5*H6;[H06,O06]=tr2rt(H06);%disp(O06)
J(4:6,1)=H00; Z00 = H00;
J(4:6,2)=H01(:,3);Z01 = H01(:,3);
J(4:6,3)=H02(:,3);Z02 = H02(:,3);
J(4:6,4)=H03(:,3);Z03 = H03(:,3);
J(4:6,5)=H04(:,3);Z04 = H04(:,3);
J(4:6,6)=H05(:,3);Z05 = H05(:,3);
%Secondly solve the linear jacobian
%transform unit from cm to m:

Jv1 = cross(Z00,(O06-O00'));J(1:3,1)=Jv1;
Jv2 = cross(Z01,(O06-O01));J(1:3,2)=Jv2;
Jv3 = cross(Z02,(O06-O02));J(1:3,3)=Jv3;
Jv4 = cross(Z03,(O06-O03));J(1:3,4)=Jv4;
Jv5 = cross(Z04,(O06-O04));J(1:3,5)=Jv5;
Jv6 = cross(Z05,(O06-O05));J(1:3,6)=Jv6;
%Then solve for analytical jacobian
%disp("H06:");
%disp(H06);
phi = atan2(H06(2,3),H06(1,3));
theta = atan2(sqrt(1-H06(3,3)*H06(3,3)),H06(3,3));
%psi = q(6);
B =zeros(3,3);
B(1,3) = cos(phi)*sin(theta); B(1,2)=-sin(phi); B(1,1)=0;
B(2,3) = sin(phi)*sin(theta); B(2,2)=cos(phi);  B(2,1)=0;
B(3,3) = cos(theta);          B(3,2)=0;         B(3,1)=1;
%disp("B:");disp(B);
Matrix66 = zeros(6,6);
Matrix66(1:3,:)=[1 0 0 0 0 0;0 1 0 0 0 0; 0 0 1 0 0 0];
Matrix66(4:6,1:3)=[0 0 0; 0 0 0; 0 0 0];
Matrix66(4:6,4:6)=inv(B);
%disp("Matrix66:");
%disp(Matrix66);
aJ=Matrix66*J;

end