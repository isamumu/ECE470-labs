function tau=att(q,qf,myrobot)
    zeta = 1;
    tau = zeros(6,1);
    H1= forwardx(q,myrobot,1);H1f = forwardx (qf, myrobot, 1);
    H2= forwardx(q,myrobot,2);H2f = forwardx (qf, myrobot, 2);
    H3= forwardx(q,myrobot,3);H3f = forwardx (qf, myrobot, 3);
    H4= forwardx(q,myrobot,4);H4f = forwardx (qf, myrobot, 4);
    H5= forwardx(q,myrobot,5);H5f = forwardx (qf, myrobot, 5);
    H6= forwardx(q,myrobot,6);H6f = forwardx (qf, myrobot, 6);
    
    o1 = H1(1:3,4);o1f = H1f(1:3,4);d1 = o1-o1f;F1 = -zeta*d1;
    o2 = H2(1:3,4);o2f = H2f(1:3,4);d2 = o2-o2f;F2 = -zeta*d2;
    o3 = H3(1:3,4);o3f = H3f(1:3,4);d3 = o3-o3f;F3 = -zeta*d3;
    o4 = H4(1:3,4);o4f = H4f(1:3,4);d4 = o4-o4f;F4 = -zeta*d4;
    o5 = H5(1:3,4);o5f = H5f(1:3,4);d5 = o5-o5f;F5 = -zeta*d5;
    o6 = H6(1:3,4);o6f = H6f(1:3,4);d6 = o6-o6f;F6 = -zeta*d6;
    
    tau1 = Jvoi(myrobot,q,1)'*F1;
    tau2 = Jvoi(myrobot,q,2)'*F2;
    tau3 = Jvoi(myrobot,q,3)'*F3;
    tau4 = Jvoi(myrobot,q,4)'*F4;
    tau5 = Jvoi(myrobot,q,5)'*F5;
    tau6 = Jvoi(myrobot,q,6)'*F6;
    tau = tau1+tau2+tau3+tau4+tau5+tau6;
    tau = tau/norm(tau);
end