function H = forward(joint, robot, i)
    H = robot.A(1:i, joint).T;
end
