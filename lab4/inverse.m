function q = inverse(H,myrobot)
    a = myrobot.a;
    d = myrobot.d;

    o = H(1:3,4);
    R = H(1:3, 1:3);
    oc = o - R * [a(6); 0; d(6)];
    
    x = oc(1);
    y = oc(2);
    z = oc(3);
    
    q = zeros(6,1);
    q(1) = atan2(y,x)-atan2(-d(2),real(sqrt(x^2+y^2-d(2)^2)));
    D = (x^2+y^2-d(2)^2+(z-d(1))^2-a(2)^2-d(4)^2)/(2*a(2)*d(4));
    q(3) = atan2(D, real(sqrt(1-D^2)));
    q(2) = atan2(z-d(1),real(sqrt(x^2+y^2-d(2)^2))) - ...
        atan2(-d(4)*cos(q(3)), a(2)+d(4)*sin(q(3)));
    H_3_0 = forward_kuka(q(1:3), myrobot);
    r_3_0 = H_3_0(1:3, 1:3);
    r_6_3 = r_3_0' * R;
    r = r_6_3;
    q(4) = atan2(r(2,3), r(1,3));
    q(5) = atan2(real(sqrt(1-r(3,3)^2)),r(3,3));
    q(6) = atan2(r(3,2), -r(3,1));
end