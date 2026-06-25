function [u, info] = uav2d_controller(t, x, scenario, p)
% Same interface for all stages. Only the meaning of the input changes.

pos = x(1:2);
vel = x(3:4);
theta = x(5);
q = x(6);

u.T = p.m*p.g;
u.tau = 0;
u.F = [0; p.m*p.g];

info.theta_d = nan;
info.pos_ref = [nan; nan];

switch scenario
    case 'attitude'
        theta_d = p.ref.theta;
        tau = p.att.kp*(theta_d - theta) - p.att.kd*q;

        u.tau = sat(tau, -p.tauMax, p.tauMax);
        u.F = [0; p.m*p.g];

        info.theta_d = theta_d;
        info.pos_ref = pos;

    case 'translation'
        [r, rdot] = uav2d_reference(t, p);
        a_cmd = p.pos.kp(:).*(r - pos) + p.pos.kd(:).*(rdot - vel);
        F = p.m*([0; p.g] + a_cmd);

        u.F = F;       % direct virtual force on the point mass
        u.T = nan;     % no thrust magnitude yet
        u.tau = 0;     % no attitude loop yet

        info.pos_ref = r;

    case 'cascade'
        [r, rdot] = uav2d_reference(t, p);
        a_cmd = p.pos.kp(:).*(r - pos) + p.pos.kd(:).*(rdot - vel);
        F = p.m*([0; p.g] + a_cmd);

        T = norm(F);
        theta_d = atan2(F(1), F(2));
        theta_d = sat(theta_d, -p.thetaMax, p.thetaMax);

        tau = p.att.kp*(theta_d - theta) - p.att.kd*q;

        u.T = sat(T, p.Tmin, p.Tmax);
        u.tau = sat(tau, -p.tauMax, p.tauMax);
        u.F = F;

        info.theta_d = theta_d;
        info.pos_ref = r;

    otherwise
        error('Unknown scenario.');
end
end

function y = sat(x, xmin, xmax)
y = min(max(x, xmin), xmax);
end
