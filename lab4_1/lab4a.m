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

for i=1:6
    subplot(2,3,i)
    %plot(t,qref);
    plot(t,q(:,i));
    s=num2str(i);
    title(['Plot for joint ' s]);
end