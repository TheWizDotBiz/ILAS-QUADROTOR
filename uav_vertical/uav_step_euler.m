function x_next = uav_step_euler(x, T, dt, p)
% uav_step_euler.m
% One Forward Euler step

dx = uav_dynamics(x, T, p);
x_next = x + dt * dx;
end
