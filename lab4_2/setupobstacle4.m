i = 0;
% Obstacle 1: Cylinder
i = i + 1;
obs{i}.R = 1/8;
obs{i}.c = [0.2;0.8];
obs{i}.rho0 = 1/4;
obs{i}.h = 2;
obs{i}.type = 'cyl';
% Obstacle 2: Sphere
i = i + 1;
obs{i}.R = 1/16;
obs{i}.c = [0.2;0.2;1.1;];
obs{i}.rho0 = 1/4;
obs{i}.type = 'sph';
% Obstacle 3: Sphere
i = i + 1;
obs{i}.R = 1/16;
obs{i}.c = [0.1;0.1;0.5;];
obs{i}.rho0 = 1/4;
obs{i}.type = 'sph';