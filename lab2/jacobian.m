function J = jacobian(q, myrobot) %q-joint angles, myrobto-structure
    theta1 = q(1);
    theta2 = q(2);
    theta3 = q(3);
    theta4 = q(4);
    theta5 = q(5);
    theta6 = q(6);
    
    J = []
    
    forward(q, myrobot); %find H of end effector wrt base
    
    for i = 1:1:6
       H_i = myrobot.links(i).A(joint(i)) 
       z = ;
       o_n = ;
       o_i = ;
    end
   
    
    
    
    
end
