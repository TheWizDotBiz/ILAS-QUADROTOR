function uav_plot(t, X, T_hist, p)
% uav_plot.m
% Simple plots for position, velocity, and thrust

z = X(1,:);
v = X(2,:);

figure;
plot(t, z, 'LineWidth', 1.5);
hold on;
yline(p.z_ref, '--');
xlabel('Time [s]');
ylabel('Position z [m]');
title('Position vs Time');
grid on;
legend('z', 'z_{ref}', 'Location', 'best');

figure;
plot(t, v, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity v [m/s]');
title('Velocity vs Time');
grid on;

figure;
plot(t, T_hist, 'LineWidth', 1.5);
hold on;
yline(p.T_hover, '--');
xlabel('Time [s]');
ylabel('Thrust T [N]');
title('Thrust vs Time');
grid on;
legend('T', 'T_{hover}', 'Location', 'best');
end
