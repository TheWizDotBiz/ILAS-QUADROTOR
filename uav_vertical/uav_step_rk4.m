function x_next = uav_step_rk4(x, T, dt, p)
% uav_step_rk4.m
% One Runge-Kutta 4 step

k1 = uav_dynamics(x, T, p);
k2 = uav_dynamics(x + 0.5 * dt * k1, T, p);
k3 = uav_dynamics(x + 0.5 * dt * k2, T, p);
k4 = uav_dynamics(x + dt * k3, T, p);

x_next = x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
end
