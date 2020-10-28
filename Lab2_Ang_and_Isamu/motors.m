function dx = motors(t,x,myrobot,splineqref, ...
                          splineqdref,splineqddref)
    % Computation of reference signal and its first two time derivatives
    
    qr=ppval(splineqref,t);
    qdr=ppval(splineqdref,t);
    qddr=ppval(splineqddref,t);

    % state vectors
    q = x(1:6);
    qd = x(7:12);
  
    for i = 1:6    
        link = myrobot.links(i);
        %disp(i);
       % disp(link.Jm);
       % J(i,1) = link.Jm;
        %B(i,1) = link.B;
    end
       J=[2e-4;2e-4;2e-4;3.3e-5;3.3e-5;3.3e-5];
       B=[1.48e-3;8.17e-3;1.38e-3;7.12e-5;8.26e-5;3.67e-5];
       %disp(J)
       %disp(B)
       
    % Controller parameters
    % Enter your gain matrices Kp and Kd here. Kp and Kd should be
    % diagonal matrices 6x6.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Kp=[8e-4 0    0    0       0       0;
        0    8e-4 0    0       0       0;
        0    0    8e-4 0       0       0;
        0    0    0    1.32e-4 0       0;
        0    0    0    0       1.32e-4 0;
        0    0    0    0       0       1.32e-4];

    Kd=[-6.8e-4 0        0      0        0        0;
        0       -0.17e-4 0      0        0        0;
        0       0        -5.8e-4 0        0        0;
        0       0        0      0.608e-4 0        0;
        0       0        0      0        0.494e-4 0;
        0       0        0      0        0        0.953e-4];
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ft = J.*qddr + B.*qdr; % feedforward term
    Vt = ft + Kd*(qdr-qd) + Kp*(qr-q);   % feedback controller
    qdd = (Vt-B.*qd)./J;   % acceleration
    
    dx = [qd; qdd;];