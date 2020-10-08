function bot = mypuma560(dh)

% DH table of puma560
L1=Link([dh(1,:) 0],'standard');
L2=Link([dh(2,:) 0],'standard');
L3=Link([dh(3,:) 0],'standard');
L4=Link([dh(4,:) 0],'standard');
L5=Link([dh(5,:) 0],'standard');
L6=Link([dh(6,:) 0],'standard');
bot = SerialLink([L1 L2 L3 L4 L5 L6]);

end