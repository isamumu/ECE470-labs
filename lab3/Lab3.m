%% definition of robot structure
  DH = [0 0.76 0 pi/2;
        0 -0.2365 0.4323 0;
        0 0 0 pi/2;
        0 0.4318 0 -pi/2;
        0 0 0 pi/2
        0 0.20 0 0];
    
myrobot = mypuma560(DH); % define our robot
%% 3.1 the attractive field
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4) = 100*[-1;3;3;]/4;
q1 = inverse(H1,myrobot)
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4) = 100*[3;-1;2;]/4;
q2 = inverse(H2,myrobot)

tau_att = att( q1,q2,myrobot )
%% 3.2 Motion Planning without obstacles
%q2(4) = q2(4) + 2*pi;
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)
%% 3.3 Motion Planning with obstacles
setupobstacle
q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1}) % This tests the torque for the cylinder obstacle
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6})

setupobstacle
hold on
axis([-10 10 -10 10 0 20])
view(-32,50)
plotobstacle(obs);
hold on
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);