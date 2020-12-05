%%
clc
clear all
disp('lab4c')
cross([0.0074;-0.1766;-0.9843],[0.0015;-0.0353;-0.1969])
%%
Lab4setup
%%
myrobot2 = perturb(myrobot,0.1);
plotobstacle(obs);
hold on
vf =@(t,x)motors4c(t,x,myrobot,myrobot2,splineqref,splineqdref,splineqddref);
q0 = [q1';zeros(6,1)];
options = odeset('AbsTol',0.01,'RelTol',0.01);
[t,q] = ode45(vf,[0 10],q0,options);
%plot(myrobot, q)
%axis([-500 500 -500 500 0 1000])
%plot(myrobot,q(:,1:6))
qref = ppval(splineqref,t);
for i=1:6
    subplot(2,3,i)
    plot(t,qref(i,:));
    hold on
    plot(t,q(:,i), 'r');
    legend({'reference signal','motor sim'})
    s=num2str(i);
    title(['Plot for joint ' s]);
end