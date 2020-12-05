%% definition of robot structure
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
setupobstacle4
q1 = inverse (H1,myrobot)
q2 = inverse (H2,myrobot)
q1= [1.5887    0.6969    0.6700   -3.1416    1.3668   -3.1236];
q2= [-0.6256    0.4241    0.5632   -3.1416    0.9873   -2.1964];
splineqref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
%%
t=linspace(0,10,300);
%q=ppval(splineqref,t)';
%plot(myrobot,q);
%qref = ppval(splineqref,t);
%for i=1:6
%    subplot(2,3,i)
%    plot(t,q(:,i));
%    hold on;
%    %plot(t,qref(i,:));
%    s=num2str(i);
%    title(['Plot for joint ' s]);
%end

t=linspace(0,10,10);
splineqref=spline(t,ppval(splineqref,t));

d=length(splineqref.coefs);
splineqdref=splineqref;
splineqdref.coefs=splineqref.coefs(:,1:3).*(ones(d,1)*[3 2 1]);
splineqdref.order=3;

d1=length(splineqdref.coefs);
splineqddref=splineqdref;
splineqddref.coefs=splineqdref.coefs(:,1:3).*(ones(d1,1)*[3 2 1]);
splineqddref.order=3;


