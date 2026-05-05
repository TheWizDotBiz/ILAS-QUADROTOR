function dx = uav_dynamics(x, T, p)
% uav_dynamics.m
% Continuous-time vertical dynamics

z = x(1); %#ok<NASGU>
v = x(2);

zdot = v;
vdot = (T - p.m * p.g) / p.m;

dx = [zdot; vdot];
end
