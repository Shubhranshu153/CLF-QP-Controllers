function [t, x] = simulate_compassgait(x0, tf)
% SIMULATE_COMPASSGAIT(x0, tf) simulates a trajectory for the compass gait
% walker on t = [0, tf], and returns cell arrays of (state,time) pairs for
% each stride simulated; i.e., (x{1},t{1}) corresponds to the first stride,
% x{2}(:,1) is the post-impact state corresponding to pre-impact state
% x{1}(:,end), and so on.
%
%   @param x0: initial state; nx by 1 vector
%   @param tf: simulation duration in seconds; scalar
%
%   @output t: cell array of timestamps for states; each t{i} is a row
%   vector.
%   @output x: cell array of state samples; each x{i} is a nx by dim(t{i})
%   matrix

% p = params();
p = params();
% p.gamma=2;
l = p.a + p.b;
t = {};
x = {};
t_cur = 0;
x_cur = x0;
i = 1;
while t_cur < tf
    options.Events = @guardEvent;
    options.InitialStep = 1e-3;
    options.Refine = 1;
    options.MaxStep = 0.02;
    [t_k,x_k,te,xe,ie] = ode45(@(t,x) f(x), [t_cur, tf], x_cur, options);
    
    t{i} = t_k(:)';
    x{i} = x_k';
    if numel(te) == 0
        break;
    end
    t_cur = te;
    x_cur = reset_map(xe');
    i = i + 1;
end
end

function [value,isterminal,direction] = guardEvent(t,x)
    value = guard(x);
    isterminal = 1;
    direction = -1;
end
