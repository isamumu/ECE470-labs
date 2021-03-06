% q0, qf column vectors representing initial and final positions
function qref=motionplan(q0,qf,t1,t2,myrobot,obs,accur)
    N = 4100; % as defined in the lab document
    alpha = 0.01; % rate of decent in gradient decent
    q = [q0];
    
    %counter=1 % will allow us to compute the while loop like for loop
    numObstacles = length(obs);
    
    for counter = 1:1:N
        counter = counter + 0 
        qc = q(end, 1:6);
        Fatt = att(qc, qf, myrobot); % compute the attractive force
        Frep = 0; % initialize the repulsive force to 0
        
        if numObstacles == 1 % if there is only one obstacle
            Frep = Frep + rep(qc, myrobot, obs);
        else % if there is multiple obstacles
            for p = 1:numObstacles
                Frep = Frep + rep(qc, myrobot, obs{p});
            end
        end
        
        q_next = qc + alpha*(Fatt' + Frep'); % transpose Frep because rep gives col vector
        q = [q; q_next];
        
        if norm(wrapTo2Pi(q_next(1:5))-wrapTo2Pi(qf(1:5))) < accur
            break
        end
        
        %counter = counter + 1 % increment for next loop
    end
    
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q'); % compute the spline for waypoints
end