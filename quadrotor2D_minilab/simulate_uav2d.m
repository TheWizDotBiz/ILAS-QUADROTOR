function sim = simulate_uav2d(scenario, p)
% Simulate one scenario with zero-order-hold control and RK4 dynamics.

t = 0:p.dt:p.tEnd;
N = numel(t);
x = zeros(6, N);
x(:,1) = p.x0;

log.T = nan(1, N);
log.tau = nan(1, N);
log.theta_d = nan(1, N);
log.F = nan(2, N);
log.pos_ref = nan(2, N);

for k = 1:N-1
    [u, info] = uav2d_controller(t(k), x(:,k), scenario, p);

    f = @(tt, xx, uu) uav2d_dynamics(tt, xx, uu, scenario, p);
    x(:,k+1) = rk4_step(f, t(k), x(:,k), u, p.dt);

    % These two cases intentionally remove part of the full model.
    if strcmp(scenario, 'attitude')
        x(1:4,k+1) = p.x0(1:4);
    elseif strcmp(scenario, 'translation')
        x(5:6,k+1) = [0; 0];
    end

    log.T(k) = u.T;
    log.tau(k) = u.tau;
    log.F(:,k) = u.F;
    log.theta_d(k) = info.theta_d;
    log.pos_ref(:,k) = info.pos_ref;
end

[u, info] = uav2d_controller(t(N), x(:,N), scenario, p);
log.T(N) = u.T;
log.tau(N) = u.tau;
log.F(:,N) = u.F;
log.theta_d(N) = info.theta_d;
log.pos_ref(:,N) = info.pos_ref;

sim.t = t;
sim.x = x;
sim.log = log;
sim.p = p;
sim.name = p.name;
end
