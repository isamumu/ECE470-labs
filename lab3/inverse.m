function q = inverse( H, myrobot )
% Inverse kinematics
q = zeros(1, 6); %will store the parameters
l1 = myrobot.links(1); % define the robotic link variables 
l2 = myrobot.links(2);
l3 = myrobot.links(3);
l4 = myrobot.links(4);
l5 = myrobot.links(5);
l6 = myrobot.links(6);

% calculation
Rd = H(1:3, 1:3); % extract rotation matrix from homogenous matrix
Od = H(1:3, 4); % extract position matrix from homogenous matrix
Oc = Od - Rd * [0; 0; l6.d]; % find origing of end effector
xc = Oc(1); % find the following axes of the end effectors
yc = Oc(2);
zc = Oc(3);

% the following calculations are based on the geometry obtained from the
% lab document

% q1
%disp(l2.d) % display for debugging
q(1) = atan2(yc, xc)- atan2(-1*l2.d, real(sqrt(xc^2 + yc^2 - l2.d^2))); % calculate theta 1
% q3
dis = xc^2 + yc^2; 
deno = (2*l2.a*l4.d);
Dd = (dis- l2.d^2 + (zc - l1.d)^2 - l2.a^2 - l4.d^2)/deno;
q(3) = atan2(Dd, real(sqrt(1 - Dd^2))); %calculate theta 3
% q2
first = atan2(zc - l1.d, real(sqrt(xc^2 + yc^2 - l2.d^2)));
second = atan2(-l4.d * cos(q(3)), l2.a + l4.d * sin(q(3)));
q(2)=first - second; % calculate theta3 as a difference of angles according to geometry

H1 = l1.A(q(1));%disp(H1);
H2 = l2.A(q(2));%disp(H2);
H3 = l3.A(q(3));%disp(H3);
Hh = H1*H2*H3;
Rr = tr2rt(Hh);
X = Rr'*Rd;

% calculate theta 4,5,6 based on the zyz euler angles
q(4) = atan2(X(2, 3), X(1, 3));
q(5) = atan2(real(sqrt(1 - X(3, 3)^2)), X(3, 3));
q(6) = atan2(X(3, 2), -X(3, 1));
end