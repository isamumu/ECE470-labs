% computes the velocity jacobian of the ith link
function J=Jvoi(robot, q, i)
    J = zeros(3,6);
    Hn = forwardx(q, robot, i); % calculate final position of ith link
    On = Hn(1:3, 4);
    for j=1:i
        Hi = forwardx(q, robot, j-1);
        Oi = Hi(1:3,4);
        Zi = Hi(1:3,3);
        J(1:3, j) = cross(Zi, On-Oi);
        if j==6
            Zi;
            O = On-Oi;
            %cross(Zi, On-Oi)
            J(1:3,j)=J(1:3,j)*(10e11);
            %J(1:3,j)
        end
    end
end