%% definition of robot structure
  DH = [0 76 0 pi/2;
        0 -23.65 43.23 0;
        0 0 0 pi/2;
        0 43.18 0 -pi/2;
        0 0 0 pi/2
        0 20 0 0];
    
myrobot = mypuma560(DH); % define our robot
%% prelab: jacobian matrix
jacobianmatrix = jacobian([pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6],myrobot) % calculate geometric jacobian
ajacobianmatrix = ajacobian([pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6],myrobot) % calculate analytic jacobian