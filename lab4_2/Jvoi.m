% computes the velocity jacobian of the ith link
function J=Jvoi(robot, q, i)
    n = length(robot.a);
    J = zeros(3,n);
    Hn = forwardx(q, robot, i); % calculate final position of ith link
    On = Hn(1:3, 4);
    for j=1:i
        Hi = forwardx(q, robot, j-1);
        Oi = Hi(1:3,4);
        Zi = Hi(1:3,3);
        J(1:3, j) = cross(Zi, On-Oi);
    end
end