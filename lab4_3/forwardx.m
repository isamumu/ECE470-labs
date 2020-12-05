function H = forwardx(joint, robot, i)
    H = robot.A(1:i, joint).T;
end
