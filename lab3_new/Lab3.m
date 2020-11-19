%% definition of robot structure
clc  
clear all
DH = [0 76 0 pi/2;
        0 -23.65 43.228 0;
        0 0 0 pi/2;
        0 43.18 0 -pi/2;
        0 0 0 pi/2
        0 20 0 0];
    
myrobot = mypuma560(DH); % define our robot
%% 3.1 the attractive field
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4) = 100*[-1;3;3;]/4;
q1 = inverse(H1,myrobot)
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4) = 100*[3;-1;2;]/4;
q2 = inverse(H2,myrobot)
%q1= [1.5887    0.6969    0.6700   -3.1416    1.3668   -3.1236];
%q2= [-0.6256    0.4241    0.5632   -3.1416    0.9873   -2.1964];
tau_att = att( q1,q2,myrobot )
%% 3.2 Motion Planning without obstacles
%q2(4) = q2(4) + 2*pi;
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)
%% 3.3.0 Motion Planning with obstacles
setupobstacle
q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1}) % This tests the torque for the cylinder obstacle
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6})
%% 3.3.1 Motion Planning with obstacles
setupobstacle
hold on
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
hold on
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);