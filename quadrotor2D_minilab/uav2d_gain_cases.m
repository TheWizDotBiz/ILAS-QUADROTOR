function P = uav2d_gain_cases(p)
% Three cases for the cascade demonstration.
%if you use the optimal tuning values we got from the two previous tasks you can
%actually optimize p1 to land perfectly at the target position and attitude
%in this situation
P(1) = p;
P(1).name = 'nominal cascade';
P(1).att.kp = 1.0; % default 0.08
P(1).att.kd = 0.25; % default 0.05
P(1).pos.kp = [1.0; 1.5]; % default [2.00; 1.00]
P(1).pos.kd = [1.775; 2.0]; % default [1.50; 1.80]

P(2) = p;
P(2).name = 'slow inner loop';
P(2).att.kp = 0.08;
P(2).att.kd = 0.05;

P(3) = p;
P(3).name = 'aggressive outer loop';
P(3).pos.kp = [2.00; 1.00];
P(3).pos.kd = [1.50; 1.80];
end
