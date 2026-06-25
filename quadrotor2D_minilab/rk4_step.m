function xNext = rk4_step(f, t, x, u, dt)
% One fixed-step RK4 update. The control input u is held constant.

k1 = f(t,          x,             u);
k2 = f(t + dt/2,  x + dt*k1/2,   u);
k3 = f(t + dt/2,  x + dt*k2/2,   u);
k4 = f(t + dt,    x + dt*k3,     u);

xNext = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end
