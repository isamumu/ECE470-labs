function [ qref ] = motionplan_rep( q0,qf,t1,t2,myrobot,obs,accur )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
q = q0;
q_temp = q0;

while (norm(q_temp(end,1:5)-qf(end,1:5))>accur)
    q_temp = q_temp + 0.01*att(q_temp,qf,myrobot) + 0.01*rep(q_temp,myrobot,obs);
    q = vertcat(q,q_temp);

t = linspace(t1,t2,size(q,1));
qref = spline(t,q');

end
