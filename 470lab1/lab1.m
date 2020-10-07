
%% 4.1 definition of robot structure
  DH = [0 76 0 pi/2;
        0 -23.65 43.23 0;
        0 0 0 pi/2;
        0 43.18 0 -pi/2;
        0 0 0 pi/2
        0 20 0 0];
myrobot = mypuma560(DH);

%% 4.2 Plot a sample joint space trajectory
q = [linspace(0, pi, 200);
     linspace(0, pi/2, 200);
     linspace(0, pi, 200);
     linspace(pi/4, 3*pi/4, 200);
     linspace(-pi/3, pi/3, 200);
     linspace(0, 2*pi, 200)];
q = q';
plot(myrobot, q)
disp("part 4.2 done")
%% 4.3 Forward Kinematics
o = zeros(200, 3);
for i = 1:200
   joint = q(i,:); 
   H = forward(joint, myrobot);
   o(i,:)=transl(H);
end
plot3(o(:,1), o(:,2), o(:,3), 'r')
hold on
plot(myrobot,q)
disp("part 4.3 done")
%% 4.4 Inverse Kinematics
 % test
 testing_m = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23;0 0 1 15;0 0 0 1];
 disp(testing_m)
 testing = inverse(testing_m, myrobot)
 disp(testing)
 disp("part 4.4.1 done")
 %% 4.4 part 2
    q = zeros(100, 6);
    d = zeros(3, 100);
    d(1,:) = linspace(10, 30, 100);  
    d(2,:) = linspace(23, 30, 100);  
    d(3,:) = linspace(15, 100, 100);
    d = d';                          
R = [cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4)  0; 0 0 1];
 for i = 1:100
     temp = d(i,:); disp(temp);
     H_ini = [R,[temp(1);temp(2);temp(3)]];
     H_fin = [H_ini; 0 0 0 1];
     q(i, :) = inverse(H_fin, myrobot);
 end
 clf;
 plot3(d(:, 1), d(:, 2), d(:, 3), 'r');
 hold on;
 plot(myrobot, q);
 disp("part 4.4.2 done")