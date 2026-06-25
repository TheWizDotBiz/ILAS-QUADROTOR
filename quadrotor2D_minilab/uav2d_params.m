function p = uav2d_params()
% Parameters kept deliberately small and readable.

p.name = 'nominal';

% Physical parameters
p.m = 1.0;        % kg thrust?/weight?
p.I = 0.02;       % kg m^2 inertia i think
p.g = 9.81;       % m/s^2 gravity dont touch that

% Simulation
p.dt = 0.02;      % sampling time [s]
p.tEnd = 8.0;     % final time [s]
p.x0 = [0; 1; 0; 0; 0; 0];   % [x z vx vz theta q]'

% References
p.ref.pos = [3; 1];           % desired [x z]'
p.ref.theta = 15*pi/180;      % desired attitude for attitude-only case

% Outer-loop position gains: virtual acceleration command
p.pos.kp = [1.0; 1.5]; % og 0.70; 1.00 temp 0.75 1.5
p.pos.kd = [1.775; 2.0]; % 0g 1.40; 1.80

% Inner-loop attitude gains: torque command
p.att.kp = 0.60; % was 0.60 opt 1.0
p.att.kd = 0.18; % was 0.18 opt 0.25

% Simple actuator/reference limits
p.thetaMax = 35*pi/180;
p.Tmin = 0;
p.Tmax = 2.5*p.m*p.g;
p.tauMax = 0.70;

% Drawing parameters
p.geom.arm = 0.30;
p.anim.skip = 2;              % draw every skip samples
p.anim.speed = 1.0;            % 1.0 means wall-clock time = simulation time
p.anim.forceScale = 0.08;
end
