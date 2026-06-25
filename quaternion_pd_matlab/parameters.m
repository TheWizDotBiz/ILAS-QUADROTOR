function P = parameters
% Quaternion convention: q = [qw qx qy qz]'.
%these default values are diagonal
P.J = diag([0.020 0.025 0.040]); %Inertia
P.Kp = diag([0.50 0.50 0.50]); %proportial gain
P.Kd = diag([0.15 0.15 0.20]); %dampener

P.dt = 0.005;
P.T = 6;

P.q0 = [1;0;0;0];
P.omega0 = zeros(3,1);

axis_d = [1;1;0.5];
axis_d = axis_d/norm(axis_d);
angle_d = 110*pi/180;
P.qd = [cos(angle_d/2); axis_d*sin(angle_d/2)];

P.axis_length = 1;
P.frame_step = 4;
P.animation_pause = 0.01;
end
