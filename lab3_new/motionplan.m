% q0, qf column vectors representing initial and final positions
function qref=motionplan(q0,qf,t1,t2,myrobot,obs,accur)
    N = 4100;
    alpha = 0.01;
    q = [q0];
    counter=1
    numObstacles = length(obs);
    while counter < N
        
        qc = q(end, 1:6);
        Fatt = att(qc, qf, myrobot);
        Frep = 0;
        if numObstacles == 1
            Frep = Frep + rep(qc, myrobot, obs);
        else
            for p = 1:numObstacles
                Frep = Frep + rep(qc, myrobot, obs{p});
            end
        end
        q_next = qc + alpha*(Fatt' + Frep'); % transpose Frep because rep gives col vector
        q = [q; q_next];
        if norm(wrapTo2Pi(q_next(1:5))-wrapTo2Pi(qf(1:5))) < accur
            break
        end
        counter = counter + 1
    end
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');
end