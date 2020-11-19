function tau = rep(q,myrobot,obs)
    tau = zeros(6,1);
    Oi = zeros(3,6);
    Frep = zeros(3,6);
    H1= forwardx(q,myrobot,1);o1 = H1(1:3,4);Oi(:,1)=o1;J1=Jvoi(myrobot, q, 1);
    H2= forwardx(q,myrobot,2);o2 = H2(1:3,4);Oi(:,2)=o2;J2=Jvoi(myrobot, q, 2);
    H3= forwardx(q,myrobot,3);o3 = H3(1:3,4);Oi(:,3)=o3;J3=Jvoi(myrobot, q, 3);
    H4= forwardx(q,myrobot,4);o4 = H4(1:3,4);Oi(:,4)=o4;J4=Jvoi(myrobot, q, 4);
    H5= forwardx(q,myrobot,5);o5 = H5(1:3,4);Oi(:,5)=o5;J5=Jvoi(myrobot, q, 5);
    H6= forwardx(q,myrobot,6);o6 = H6(1:3,4);Oi(:,6)=o6;J6=Jvoi(myrobot, q, 6);
    for i = 1:6
        o_i =Oi(:,i);  
        if strcmp(obs.type,'cyl')
            b_pre = obs.c + obs.R*(o_i(1:2) - obs.c)/norm(o_i(1:2) - obs.c);
            b = [b_pre; o_i(3)];
            rho = norm(o_i - b);
            gradient = (o_i - b)/rho;
        elseif strcmp(obs.type,'sph')
            b = obs.c + obs.R*(o_i - obs.c)/norm(o_i - obs.c);
            rho = norm(o_i - b);
            gradient = (o_i - b)/rho;
        end
        if rho <= obs.rho0
            F_rep = (1/rho - 1/obs.rho0)*(1/rho^2)*gradient;
        else
            F_rep = zeros(3,1);
        end
        Frep(:,i)=F_rep;
        
    end
    tau1 = J1'*Frep(:,1);
    tau2 = J2'*Frep(:,2);
    tau3 = J3'*Frep(:,3);
    tau4 = J4'*Frep(:,4);
    tau5 = J5'*Frep(:,5);
    tau6 = J6'*Frep(:,6);
    tau = tau1+tau2+tau3+tau4+tau5+tau6;
    %tau = tau + J'*F_rep;
    if norm(tau) ~= 0
        tau = tau/norm(tau);
    end
end