%%
clc
clear all
disp('lab4a')
Lab4setup
%%
setupobstacle4
plotobstacle(obs);
hold on
vf =@(t,x)motors4a(t,x,myrobot,splineqref,splineqdref,splineqddref);

q0 = [q1';zeros(6,1)];
options = odeset('AbsTol',0.01,'RelTol',0.01);
[t,q] = ode45(vf,[0 10],q0,options);
%plot(myrobot, q)
%axis([-500 500 -500 500 0 1000])
plot(myrobot,q(:,1:6))

DH = [0 0.76 0 pi/2;
        0 -0.2365 0.43228 0;
        0 0 0 pi/2;
        0 0.4318 0 -pi/2;
        0 0 0 pi/2
        0 0.20 0 0];
    
myrobot = mypuma560(DH); % define our robot
myrobot.links(1).Jm
%% 4.1.2 creating lab4setup.m
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4)=[-1; 3; 3;]/4;
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=[3; -1; 2;]/4;
%setupobstacle4

q1 = inverse (H1,myrobot)
q2 = inverse (H2,myrobot)
splineqref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
qref = ppval(splineqref,t);

for i=1:6
    subplot(2,3,i)
    plot(t,q(:,i));
    hold on;
    plot(t,qref(i,:));
    s=num2str(i);
    title(['Plot for joint ' s]);
end
