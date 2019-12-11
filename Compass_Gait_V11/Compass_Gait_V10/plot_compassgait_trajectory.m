function plot_compassgait_trajectory(t, x)
%PLOT_COMPASSGAIT_TRAJECTORY(t,x) visualizes a trajectory output from
%simulate_compassgait.
%
%   @param t: cell array of timestamps for states; each t{i} is a row
%   vector.
%   @param x: cell array of state samples; each x{i} is a nx by dim(t{i})
%   matrix
filename = 'just_initial_error.gif';
par = params();
a = par.a;
b = par.b;
l = a + b; 
gamma = par.gamma;
% gamma = deg2rad(3);

% combined times
t_t = zeros(1,0);

% combined states
x_t = zeros(4,0);

% combined stance-foot locations
p_t = zeros(2,0);

% combined modes (0 for stance foot ~ left foot, 1 for stance ~ right)
m_t = zeros(1,0);

p_cur = [0;0];
for i=1:numel(t)
   t_t = [t_t t{i}];
   x_t = [x_t x{i}];
   p_t = [p_t repmat(p_cur,1,numel(t{i}))];
   m_t = [m_t repmat(mod(i,2),1,numel(t{i}))];
   
   % calculate next stance foot location
   hip = p_cur + l*[-sin(x_t(2,end)); cos(x_t(2,end))];
   p_cur = hip - l*[-sin(x_t(1,end)); cos(x_t(1,end))];
end

stale = .01;
tic

i = 1;



while i<=numel(t_t)
    start = toc;
    
    % get current states, locations
    x = x_t(:,i);
    stfoot = p_t(:,i);
    q = x(1:2);

    h = figure(25);
    set(h,'DoubleBuffer','on');
    clf;
    hold on;
    
    % find hip and swing foot positions
    hip = stfoot + l*[-sin(q(2)); cos(q(2))];
    swfoot = hip - l*[-sin(q(1)); cos(q(1))];
    
    % draw ground
    x = hip(1) + 1*[-1,1];
    line(x, tan(-gamma)*x,'Color',[0.5 0.5 0.5],'LineWidth',1.5);
    
    % draw legs
    line([stfoot(1), hip(1)], ...
        [stfoot(2), hip(2)],'LineWidth',2.0,'Color','k');
    line([hip(1), swfoot(1)], ...
        [hip(2), swfoot(2)],'LineWidth',2.0,'Color','k');
    
    % draw masses, putting green on the left leg and red on the right
    circ = 0:0.1:2*pi;
    m = m_t(i);
    lfoot = m*stfoot + (1-m)*swfoot;
    rfoot = m*swfoot + (1-m)*stfoot;
    lq = m*2 + (1-m)*1;
    rq = m*1 + (1-m)*2;
    fill(hip(1) + 0.05*sin(circ), hip(2) + 0.05*cos(circ),[0 0 1]);
    fill(lfoot(1) -a*sin(q(lq)) + 0.05*sin(circ), ...
        lfoot(2) + a*cos(q(lq)) + 0.05*cos(circ),[0 1 0]);
    fill(rfoot(1) -a*sin(q(rq)) + 0.05*sin(circ), ...
        rfoot(2) + a*cos(q(rq)) + 0.05*cos(circ),[1 0 0]);

    axis equal;
    %  axis off;
    hold off;
    
    
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i==1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.12); 
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.12 ); 
    end

    compu = toc - start;
    stale_i = max(stale,compu*2);
    next_i = find(t_t >= start + stale_i);
    if numel(next_i) < 1
        if i < numel(t_t)
            i = numel(t_t);
        else
            break;
        end
    else
        i = next_i(1);
    end
    pause(t_t(i) - toc-1.8);
%     pause(0.1)
%     t_t(i)-toc

end

end