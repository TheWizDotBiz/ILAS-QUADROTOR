function uav_animate(t, z_hist)
% uav_animate.m
% Simple vertical motion animation

figure;
axis equal;
xlim([-1 1]);
ymin = min([0, z_hist]) - 0.5;
ymax = max([1, z_hist]) + 0.5;
ylim([ymin ymax]);
grid on;
xlabel('x');
ylabel('z [m]');
title('Vertical Motion Animation');

hold on;
plot([-0.5 0.5], [0 0], 'k-', 'LineWidth', 2); % ground
h = plot(0, z_hist(1), 'o', 'MarkerSize', 10, 'LineWidth', 2);

for k = 1:length(t)
    set(h, 'YData', z_hist(k));
    drawnow;
    pause(0.02);
end
end
