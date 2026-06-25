function animate_uav2d(sims, scenario)
% Animation only. Nothing here is needed by the controller or the dynamics.

nS = numel(sims);
p = sims(1).p;
t = sims(1).t;

[xlimv, ylimv] = animation_limits(sims, scenario);

figure('Color','w','Name',['2D UAV animation: ', scenario]);
col = lines(max(nS,3));

for i = 1:nS
    if nS == 1
        ax(i) = axes; %#ok<AGROW,LAXES>
    else
        ax(i) = subplot(1, nS, i); %#ok<AGROW,LAXES>
    end

    hold(ax(i),'on'); grid(ax(i),'on'); box(ax(i),'on');
    axis(ax(i),'equal');
    xlim(ax(i), xlimv); ylim(ax(i), ylimv);
    xlabel(ax(i),'x [m]'); ylabel(ax(i),'z [m]');
    title(ax(i), sims(i).name, 'Interpreter','none');

    plot(ax(i), xlimv, [0 0], 'k-', 'LineWidth', 1);

    if ~strcmp(scenario, 'attitude')
        r = sims(i).p.ref.pos;
        plot(ax(i), r(1), r(2), 'x', 'Color', col(2,:), ...
            'MarkerSize', 10, 'LineWidth', 2);
    end

    h{i} = init_vehicle(ax(i), col(i,:)); %#ok<AGROW>
    h{i}.trail = plot(ax(i), nan, nan, '-', 'Color', col(i,:), 'LineWidth', 1.2);
    h{i}.clock = text(ax(i), xlimv(1)+0.04*diff(xlimv), ...
        ylimv(2)-0.08*diff(ylimv), '', 'FontWeight','bold');
end

skip = max(1, p.anim.skip);
speed = max(eps, p.anim.speed);

tic
for k = 1:skip:numel(t)
    while toc < t(k)/speed
        pause(0.001);
    end

    for i = 1:nS
        update_vehicle(h{i}, sims(i), k, scenario);
        set(h{i}.trail, 'XData', sims(i).x(1,1:k), 'YData', sims(i).x(2,1:k));
        set(h{i}.clock, 'String', sprintf('t = %.2f s', sims(i).t(k)));
    end
    drawnow limitrate
end
end

function h = init_vehicle(ax, color)
h.body = plot(ax, nan, nan, '-', 'Color', color, 'LineWidth', 3);
h.leftRotor = plot(ax, nan, nan, 'o', 'Color', color, ...
    'MarkerFaceColor', 'w', 'MarkerSize', 8, 'LineWidth', 1.5);
h.rightRotor = plot(ax, nan, nan, 'o', 'Color', color, ...
    'MarkerFaceColor', 'w', 'MarkerSize', 8, 'LineWidth', 1.5);
h.center = plot(ax, nan, nan, 'o', 'Color', color, ...
    'MarkerFaceColor', color, 'MarkerSize', 5);
h.force = quiver(ax, 0, 0, 0, 0, 0, 'Color', color, ...
    'LineWidth', 1.2, 'MaxHeadSize', 1.8);
h.desired = plot(ax, nan, nan, ':', 'Color', color, 'LineWidth', 1.5);
end

function update_vehicle(h, sim, k, scenario)
p = sim.p;
x = sim.x(:,k);
pos = x(1:2);
theta = x(5);

if strcmp(scenario, 'translation')
    theta = 0;   % this is the modelling assumption in Stage 2
end

ex = [cos(theta); -sin(theta)];
left = pos - p.geom.arm*ex;
right = pos + p.geom.arm*ex;

set(h.body, 'XData', [left(1) right(1)], 'YData', [left(2) right(2)]);
set(h.leftRotor, 'XData', left(1), 'YData', left(2));
set(h.rightRotor, 'XData', right(1), 'YData', right(2));
set(h.center, 'XData', pos(1), 'YData', pos(2));

switch scenario
    case 'translation'
        Fdraw = sim.log.F(:,k);
    case 'cascade'
        Fdraw = sim.log.T(k)*[sin(theta); cos(theta)];
    otherwise
        Fdraw = p.m*p.g*[sin(theta); cos(theta)];
end

Fdraw = p.anim.forceScale*Fdraw;
set(h.force, 'XData', pos(1), 'YData', pos(2), ...
    'UData', Fdraw(1), 'VData', Fdraw(2));

if strcmp(scenario, 'cascade') && isfinite(sim.log.theta_d(k))
    theta_d = sim.log.theta_d(k);
    ed = [cos(theta_d); -sin(theta_d)];
    a = pos - 0.75*p.geom.arm*ed;
    b = pos + 0.75*p.geom.arm*ed;
    set(h.desired, 'XData', [a(1) b(1)], 'YData', [a(2) b(2)]);
else
    set(h.desired, 'XData', nan, 'YData', nan);
end
end

function [xlimv, ylimv] = animation_limits(sims, scenario)
xs = [];
zs = [];
for i = 1:numel(sims)
    xs = [xs sims(i).x(1,:)]; %#ok<AGROW>
    zs = [zs sims(i).x(2,:)]; %#ok<AGROW>
    if ~strcmp(scenario, 'attitude')
        xs = [xs sims(i).p.ref.pos(1)]; %#ok<AGROW>
        zs = [zs sims(i).p.ref.pos(2)]; %#ok<AGROW>
    end
end

margin = 0.8;
xlimv = [min(xs)-margin, max(xs)+margin];
ylimv = [max(0, min(zs)-margin), max(zs)+margin];

if diff(xlimv) < 2
    mid = mean(xlimv);
    xlimv = [mid-1, mid+1];
end
if diff(ylimv) < 2
    mid = mean(ylimv);
    ylimv = [mid-1, mid+1];
end
end
