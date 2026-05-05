%% main_uav_vertical.m
% Lecture 3: UAV vertical dynamics, simulation, and proportional control
clear; clc; close all;

%% 1) Load parameters
p = uav_params();
%params are contained within uav_params.m, go mess with em in said file

%% 2) Create time vector
N = floor(p.t_final / p.dt) + 1;
t = (0:N-1) * p.dt;

%% 3) Initial state
% x = [z; v]
x = [p.z0; p.v0];

%% 4) Preallocate memory
X = zeros(2, N);
T_hist = zeros(1, N);
X(:,1) = x;

e_int = 0;

%% 5) Simulation loop
for k = 1:N-1

    % -------------------------------------------------
    % Choose thrust input
    % Uncomment ONE case only
    % -------------------------------------------------

    % Open-loop cases
    %T = p.T_hover;           % hover
    % T = 1.10 * p.T_hover;    % upward acceleration
    % T = 0.90 * p.T_hover;    % downward acceleration

    % Closed-loop case: proportional control
    e = p.z_ref - x(1);
    e_int = e_int + e * p.dt; %error integral, sum it up every loop.
    %e * p.dt is a 'rectangle', width is time, height is error.
    %T = p.T_hover + p.Kp * e;
    %T = p.T_hover + p.Kp * e - 2 * x(2); %- 2 * x(2) is proportial dampener iirc
    %T = p.T_hover + p.Kp * e - p.Kd * x(2);
    T = p.T_hover + p.Kp * e - p.Kd * x(2) + p.Ki * e_int; %this apparently is the main alg we usin this semester. ig.
    % Store thrust
    T_hist(k) = T;

    % -------------------------------------------------
    % Choose numerical method
    % Uncomment ONE line only
    % -------------------------------------------------

    %x_next = uav_step_euler(x, T, p.dt, p);
    x_next = uav_step_rk4(x, T, p.dt, p);

    % Update state
    x = x_next;
    X(:,k+1) = x;
end

% Store last thrust value for plotting
T_hist(N) = T_hist(N-1);

%% 6) Plot results
uav_plot(t, X, T_hist, p);

%% 7) Animate motion
uav_animate(t, X(1,:));
