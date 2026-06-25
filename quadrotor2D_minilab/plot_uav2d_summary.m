function plot_uav2d_summary(sims, scenario)
% Small diagnostic plots for lecture discussion and gain comparison.

nS = numel(sims);
col = lines(max(nS,3));

figure('Color','w','Name',['2D UAV summary: ', scenario]);

subplot(2,1,1); hold on; grid on; box on; axis equal;
xlabel('x [m]'); ylabel('z [m]'); title('Trajectory');
for i = 1:nS
    plot(sims(i).x(1,:), sims(i).x(2,:), 'LineWidth', 1.5, 'Color', col(i,:));
end
if ~strcmp(scenario, 'attitude')
    plot(sims(1).p.ref.pos(1), sims(1).p.ref.pos(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
end
legend({sims.name}, 'Location','best', 'Interpreter','none');

subplot(2,1,2); hold on; grid on; box on;
xlabel('time [s]');

switch scenario
    case 'attitude'
        ylabel('theta [deg]'); title('Attitude response');
        for i = 1:nS
            plot(sims(i).t, 180/pi*sims(i).x(5,:), 'LineWidth', 1.5, 'Color', col(i,:));
            plot(sims(i).t, 180/pi*sims(i).log.theta_d, '--', 'Color', col(i,:));
        end

    case 'translation'
        ylabel('position [m]'); title('Point-mass position response');
        plot(sims(1).t, sims(1).x(1,:), 'LineWidth', 1.5);
        plot(sims(1).t, sims(1).x(2,:), 'LineWidth', 1.5);
        legend('x','z', 'Location','best');

    otherwise
        ylabel('theta [deg]'); title('Actual attitude and commanded attitude');
        for i = 1:nS
            plot(sims(i).t, 180/pi*sims(i).x(5,:), 'LineWidth', 1.5, 'Color', col(i,:));
            plot(sims(i).t, 180/pi*sims(i).log.theta_d, '--', 'Color', col(i,:));
        end
end
end
