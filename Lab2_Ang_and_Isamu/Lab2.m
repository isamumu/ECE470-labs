%% definition of robot structure
  DH = [0 0.76 0 pi/2;
        0 -0.2365 0.4323 0;
        0 0 0 pi/2;
        0 0.4318 0 -pi/2;
        0 0 0 pi/2
        0 0.20 0 0];
    
myrobot = mypuma560(DH); % define our robot
%% prelab: jacobian matrix
jacobianmatrix = jacobian([pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6],myrobot)
ajacobianmatrix = ajacobian([pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6],myrobot)

%% 4.1
%4.1.1
qref = zeros(6,100);
qdref = zeros(6,100);
qdref1 = zeros(6,100);
norm_vec = zeros(100);

 for i = 1:100
   temp = inverse(Href(:,:,i), myrobot);
   qref(:,i)=temp';
   %disp(temp');
 end
 
 %4.1.2 for each i btw 1 to 100
 for i = 1:100
     H = Href(:,:,i);
     Hdot = Hrefdot(:,:,i);
     qrefi = qref(:,i);
     odot = Hdot(1:3,4);
     R0n = H(1:3,1:3);
     R0ndot = Hdot(1:3,1:3);
     s_omega = R0ndot*(R0n');
     omega_x = s_omega(3,2);
     omega_y = s_omega(1,3);
     omega_z = s_omega(2,1);
     omega = [omega_x;omega_y;omega_z];
     
     %4.1.2.b calculating qdref
     odotomega = [odot;omega];
     Jqref = jacobian(qrefi,myrobot);
     qrefdot = (inv(Jqref))*odotomega;
     qdref(:,i)=qrefdot;
     
     %4.1.2.c calculating eul
     eul = tr2eul(H);
     phi = eul(1); theta = eul(2);
     B=[0 -sin(phi) cos(phi)*sin(theta); 0 cos(phi) sin(phi)*sin(theta); 1 0 cos(theta)];
     
     %4.1.2.d calculating alphadot
     alphadot = inv(B)*omega;
     
     %4.1.2.e calculating qdref1
     odotalphadot = [odot;alphadot];
     aJqref = ajacobian(qrefi,myrobot);
     qrefdot1 = (inv(aJqref))*odotalphadot;
     qdref1(:,i)=qrefdot1;
     
     %4.1.2.f check for correctness
     diff = qrefdot - qrefdot1;
     normdiff = norm(diff);
     norm_vec(i)=normdiff;
     %disp(i);
     %disp(sin(theta));
     %disp(normdiff);
 end
 %disp(norm_vec);
 %disp(qdref);
 %disp(qdref1);
 
 %4.1.3 cubic splines
 splineqref=spline(t,qref);
 splineqdref=spline(t,qdref);
 
 %4.1.4 more derivative using splineqddref
 d=length(splineqdref.coefs);
 splineqddref=splineqdref;
 splineqddref.coefs=splineqdref.coefs(:,1:3).*(ones(d,1)*[3 2 1]);
 splineqddref.order=3;

%% 4.2 test independent joint controller
%4.2.2
sys=@(t,x)motors(t,x,myrobot,splineqref,splineqdref,splineqddref);
Ts=0.02;
q0=[3*pi/2;zeros(11,1)];
[t,q]=ode45(sys,[0:Ts:6*pi]',q0);
%generate 6 figures

for i=1:6
    subplot(2,3,i)
    plot(t,q(:,i));
    s=num2str(i);
    title(['Plot for joint ' s]);
end
%4.2.3
hold on;
figure(2);
oref=squeeze(Href(1:3,4,:));
plot3(oref(1,:),oref(2,:),oref(3,:),'r')
view(-125,40);
plot(myrobot,q(:,1:6))
hold off
%% 