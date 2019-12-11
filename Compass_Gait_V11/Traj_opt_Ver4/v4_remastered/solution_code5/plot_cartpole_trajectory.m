function plot_cartpole_trajectory(t, x)
y_d = 0*t;
z_d = 0*t;

th = x(:,2)';
x = x(:,1)';

B = 2;
xrange = [min(x) - B, max(x) + B];
yrange = [-1, 3];
tic

L = 1;
h = .2;
w = .4;
pend = .1;
pennblue = [1,37,110]/256;
pennred = [149,0,26]/256;
px = x + L*sin(th);
py = - L*cos(th);


stale = .01;
tic

i = 1;

while i<=numel(t)
    start = toc;
    hold off;
     
    plot(4*xrange,[0 0], 'k', 'LineWidth',3)
    hold on;
    rectangle('Position',[x(i)-w/2, -h/2, w, h],'FaceColor',pennblue,'EdgeColor','k',...
    'LineWidth',3)

    plot([x(i), x(i) + L*sin(th(i))],[0, -L*cos(th(i))], 'k', 'LineWidth',3);
    
    rectangle('Position',[x(i) + L*sin(th(i))-pend/2,-L*cos(th(i))-pend/2,pend,pend],...
        'Curvature',[1,1], 'FaceColor',pennred,'EdgeColor','k','LineWidth',3);
    plot(px(1:i), py(1:i), 'g','LineWidth',3);
    axis equal;
    %xlim([x(i) - 2, x(i) + 2]);
    xlim(xrange);
    ylim(yrange);
    %legend('x_d(t)','x(t)');
    xlabel('y');
    ylabel('z');
    titl = sprintf('Cart-Pole Trajectory, $t =  %.2f $',t(i));
    title(titl,'Interpreter','latex');
    
    
    compu = toc - start;
    stale_i = max(stale,compu*2);
    next_i = find(t >= start + stale_i);
    if numel(next_i) < 1
        if i < numel(t)
            i = numel(t);
        else
            break;
        end
    else
        i = next_i(1);
    end
    pause(t(i) - toc);

end