function dx = motors4b(t,x,myrobot,splineqref,splineqdref,splineqddref)

    % Speed up time to reduce overall simulation time
    %t = t*10;
    
    % Computation of reference signal and its first two time derivatives
    qref = ppval(splineqref,t);
    qdref = ppval(splineqdref,t);
    qddref = ppval(splineqddref,t);

    % State vectors
    q = x(1:6);
    qd = x(7:12);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Edit below here
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Insert Kp and Kd here
    
    
    Kp=[4 0    0    0       0       0;
        0    4 0    0       0       0;
        0    0    4 0       0       0;
        0    0    0    4 0       0;
        0    0    0    0       4 0;
        0    0    0    0       0       4];

    Kd=[4 0        0      0        0        0;
        0       4 0      0        0        0;
        0       0        4 0        0        0;
        0       0        0      4 0        0;
        0       0        0      0        4 0;
        0       0        0      0        0        4];

    % Outer Loop Controller, insert equation (8.25) here
    aq = qddref - Kp*(q-qref)-Kd*(qd - qdref);
    % Inner Loop Controller, insert equation (8.32) here
    u = rne(myrobot,q',qd',aq');
    
    if (t>=5)&&(t<=7)
        F = [-500;1000;1000];
        Jaco = jacobian (q,myrobot);
        u_plus = Jaco(1:3,:)'*F;
        
        u = u + u_plus';
    end
    
    % Insert dynamic model of the robot here
    qdd = accel(myrobot,q',qd',u);
    disp(t)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Edit above here
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    dx = [qd; qdd;];
      
end