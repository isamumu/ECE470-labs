function H = forward( joint, myrobot )

% forward kinemetic sovler 
H1 = myrobot.links(1).A(joint(1));%disp(H1);
H2 = myrobot.links(2).A(joint(2));%disp(H2);
H3 = myrobot.links(3).A(joint(3));%disp(H3);
H4 = myrobot.links(4).A(joint(4));%disp(H4);
H5 = myrobot.links(5).A(joint(5));%disp(H5);
H6 = myrobot.links(6).A(joint(6));%disp(H6);
H = H1*H2*H3*H4*H5*H6; % calculate the total homogeneous matrix

end