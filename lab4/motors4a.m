function dx = motors4a(t,x,myrobot,splineqref,splineqdref,splineqddref)

    % Speed up time to reduce overall simulation time
    t = t*10;
    
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
    Kp=[8e-4 0    0    0       0       0;
        0    8e-4 0    0       0       0;
        0    0    8e-4 0       0       0;
        0    0    0    1.32e-4 0       0;
        0    0    0    0       1.32e-4 0;
        0    0    0    0       0       1.32e-4];

    Kd=[-6.8e-4 0        0      0        0        0;
        0       -0.17e-4 0      0        0        0;
        0       0        5.8e-4 0        0        0;
        0       0        0      0.608e-4 0        0;
        0       0        0      0        0.494e-4 0;
        0       0        0      0        0        0.953e-4];

    % Outer Loop Controller, insert equation (8.25) here
    aq = qddref - Kp*(q-qref)-Kd*(qd - qdref);
    % Inner Loop Controller, insert equation (8.32) here
    u = rne(myrobot,q',qd',aq');
    % Insert dynamic model of the robot here
    qdd = accel(myrobot,q',qd',u);
    disp('pp')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Edit above here
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    dx = [qd; qdd;];
      
end