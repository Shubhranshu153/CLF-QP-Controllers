function [t, x, u] = simulate_cartpole
% SIMULATE_CARTPOLE simulates a trajectory for the cartpole system
%
%   @output t: timestamps for states and inputs
%   @outout x: state trajectory
%   @outout u: input trajectory

N = 60;
dt = 6/N;
nx = 4;
nu = 1;

x_0 = [0; 0; 0; 0];
x_f = [0; pi; 0; 0];

z = find_swingup_trajectory(x_0, x_f, N, dt);

u_t = 0:dt:(dt*(N-1));
u = zeros(nu,0);
x_dc = zeros(nx,0);
for i=1:N
   x_i_inds = (1:nu) + (nx + nu) * (i - 1);
   u_i_inds = (1:nu) + nx * i + nu * (i - 1);
   
   x_dc(:,i) = z(x_i_inds);
   u(:,i) = z(u_i_inds);
end



A_mat = reshape([0.0,0.0,0.0,0.0,0.0, ...
                 0.0,9.81e2./1.0e2,9.81e2./5.0e1,...
                 1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],[4,4]);
B_mat = [0.0;0.0;1.0;1.0];
[K, S] = lqr(A_mat,B_mat,eye(nx),eye(nu));
        
opts = odeset('MaxStep', 0.1,'RelTol',1e-4,'AbsTol',1e-4);

[t, x] = ode45(@(t,x) dynamics(t, x, u, dt, K, S), [0 dt*(N)*1.5], x_0, opts);

plot_cartpole_trajectory(t, x);

t = u_t
x = x_dc

end

function dx = dynamics(t, x, u, dt, K, S)
    g = 9.81;
    dist = x(4)^2 + (x(2)-pi)^2;
    x_star = [0;pi;0;0];
    
    % if state is close to swing-up equilibrium, let LQR take over
    % use cost-to-go as a metric of closeness
    if (x-x_star)'*S*(x-x_star) < 3
        u_x = - K*(x-x_star);
    else
        % find the right spline i to sample from
        i = ceil(t/dt);
        if i == 0
            i = 1;
        end
        if i >= numel(u)
            i = numel(u) - 1;
        end
        lambda = mod(t,dt)/dt;
        
        % time is between u(i) and u(i+1); select input as r_i(t)
        u_x = u(i)*(1-lambda) + u(i+1)*lambda;
    end
    
    dx = f(x, u_x);
end
