function animate_attitude(t,q,P)
L = P.axis_length;
Rd = quaternion_to_rotation(P.qd);

fig = figure('Color','w','Name','Quaternion PD attitude control');
ax = axes(fig);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
axis(ax,1.25*[-1 1 -1 1 -1 1]);
view(ax,35,25);
xlabel(ax,'x_W'); ylabel(ax,'y_W'); zlabel(ax,'z_W');

plot_axis(ax,L*Rd(:,1),'r--');
plot_axis(ax,L*Rd(:,2),'g--');
plot_axis(ax,L*Rd(:,3),'b--');

R = quaternion_to_rotation(q(:,1));
hx = quiver_axis(ax,L*R(:,1),'r');
hy = quiver_axis(ax,L*R(:,2),'g');
hz = quiver_axis(ax,L*R(:,3),'b');

tx = text(ax,1.10*L*R(1,1),1.10*L*R(2,1),1.10*L*R(3,1),'x_B','Color','r','FontWeight','bold');
ty = text(ax,1.10*L*R(1,2),1.10*L*R(2,2),1.10*L*R(3,2),'y_B','Color','g','FontWeight','bold');
tz = text(ax,1.10*L*R(1,3),1.10*L*R(2,3),1.10*L*R(3,3),'z_B','Color','b','FontWeight','bold');

indices = unique([1:P.frame_step:numel(t),numel(t)]);

for k = indices
    R = quaternion_to_rotation(q(:,k));

    update_axis(hx,L*R(:,1));
    update_axis(hy,L*R(:,2));
    update_axis(hz,L*R(:,3));

    update_label(tx,1.10*L*R(:,1));
    update_label(ty,1.10*L*R(:,2));
    update_label(tz,1.10*L*R(:,3));

    c = min(1,abs(P.qd.'*q(:,k)));
    error_deg = 2*acos(c)*180/pi;
    title(ax,{sprintf('Quaternion PD control   t = %.2f s   error = %.1f deg',t(k),error_deg), ...
              'solid: current axes    dashed: desired axes'});

    drawnow;
    pause(P.animation_pause);
end
end

function h = quiver_axis(ax,v,color)
h = quiver3(ax,0,0,0,v(1),v(2),v(3), ...
    'Color',color,'AutoScale','off','LineWidth',2.5,'MaxHeadSize',0.25);
end

function plot_axis(ax,v,style)
plot3(ax,[0 v(1)],[0 v(2)],[0 v(3)],style,'LineWidth',1.5);
end

function update_axis(h,v)
set(h,'UData',v(1),'VData',v(2),'WData',v(3));
end

function update_label(h,v)
set(h,'Position',v.');
end

function R = quaternion_to_rotation(q)
q = q/norm(q);
w = q(1); x = q(2); y = q(3); z = q(4);

R = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
     2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
     2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
end
