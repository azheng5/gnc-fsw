x0 = [0,1;0,0;0,0];
y0 = [0,0;0,1;0,0];
z0 = [0,0;0,0;0,1];
psi = 45;
theta = 45;
phi = 45;

yaw = [cosd(psi), -sind(psi), 0; sind(psi), cosd(psi), 0; 0, 0, 1];
pitch = [cosd(theta), 0, sind(theta); 0, 1, 0; -sind(theta), 0, cosd(theta)];
roll = [1, 0, 0; 0, cosd(phi), -sind(phi); 0 sind(phi), cosd(phi)];

rot_matrix = yaw * pitch * roll;
x1 = rot_matrix * x0
y1 = rot_matrix * y0
z1 = rot_matrix * z0

