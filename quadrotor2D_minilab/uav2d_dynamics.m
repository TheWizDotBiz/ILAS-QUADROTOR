function xdot = uav2d_dynamics(~, x, u, scenario, p)
% State: x = [px pz vx vz theta q]'.  z is positive upward.

xdot = zeros(6,1);

switch scenario
    case 'attitude'
        % I*theta_ddot = tau. Translation is removed on purpose.
        xdot(5) = x(6);
        xdot(6) = u.tau/p.I;

    case 'translation'
        % Point mass with direct virtual force input F = [Fx Fz]'.
        xdot(1) = x(3);
        xdot(2) = x(4);
        xdot(3) = u.F(1)/p.m;
        xdot(4) = u.F(2)/p.m - p.g;

    case 'cascade'
        % Full planar quadrotor model used in Lecture 6.
        theta = x(5);
        xdot(1) = x(3);
        xdot(2) = x(4);
        xdot(3) = u.T*sin(theta)/p.m;
        xdot(4) = u.T*cos(theta)/p.m - p.g;
        xdot(5) = x(6);
        xdot(6) = u.tau/p.I;

    otherwise
        error('Unknown scenario.');
end
end
