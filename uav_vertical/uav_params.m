function p = uav_params()
% uav_params.m
% Physical and simulation parameters
% modify stuff in there to mess with this uav_vertical projects output
% inertia does not seem to be taken in consideration in this project
% compensate intertia using dampening

p.m = 1.0;          % mass [kg] T, i think? default is 1.0
p.g = 9.81;         % gravity [m/s^2] mg
p.alpha = 1.1; % If 1 (default), hover, if greater, rise, if lower, fall. Multiplies the thrust basically.
%Ierr = 0.1; %intentional error
%p.T_hover = p.m * p.g;
p.T_hover = p.m * p.g * p.alpha;

p.dt = 0.01;        % sampling time [s] deltatime
p.t_final = 10.0;    % simulation end time [s]

p.z0 = 0.0;         % initial position [m]
p.v0 = 0.0;         % initial velocity [m/s]

p.z_ref = 1.0;      % target altitude [m]
p.Kp = 1.0;         % proportional gain %%modify these three values to make the drone follow z_ref as closely as possible.
p.Kd = 1.0;         % derivative gain. default 1.0 %%this is called a PiD controller btw
p.Ki = 1.0;         % integral gain
%default params for Kp Kd and Ki are 1.0/1.0/0.5
end
