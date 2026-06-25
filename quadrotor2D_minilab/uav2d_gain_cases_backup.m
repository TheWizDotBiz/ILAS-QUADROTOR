function P = uav2d_gain_cases(p)
% Three cases for the cascade demonstration.

P(1) = p;
P(1).name = 'nominal cascade';

P(2) = p;
P(2).name = 'slow inner loop';
P(2).att.kp = 0.08;
P(2).att.kd = 0.05;

P(3) = p;
P(3).name = 'aggressive outer loop';
P(3).pos.kp = [2.00; 1.00];
P(3).pos.kd = [1.50; 1.80];
end
