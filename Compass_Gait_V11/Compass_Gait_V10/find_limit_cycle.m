function [t, x] = find_limit_cycle
%FIND_LIMIT_CYCLE executes a direct collocation optimization program to
%find a limit cycle for the compass gait walker with N sample points.
%
%   @output t: row vector with indices corresponding to the time of the
%   i-th state sample; 1 by N vector
%   @output x: matrix with columns corresponding to x_i, the ith state
%   sample in the limit cycle; 4 by N matrix

    nx = 4; %seems to be number of DOF
    
    N = 100;
    nz = N * (nx + 1) - 1;

    
    % get initial guess with simulation
    x0 = [-.326, .22, -.381, -1.1]';%+[0.1;0;0.05/2.1;0.1]*2; %we need something more concrete, Now plugging some random values
    [t, x] = simulate_compassgait(x0, 4); %ode is being called
    x0_spline = spline(t{1},x{1});
    t0 = linspace(t{1}(1),t{1}(end),N);
    x0 = ppval(x0_spline, t0);
    dt_0 = t0(2:end) - t0(1:end-1);
    
    
    % set bounds on dt's to not stray far from simulation
    lb = -inf(nz, 1);
    ub = inf(nz, 1);
    
    dtavg = t{1}(end)/(N-1);
    dtmin = 0.25*dtavg;
    dtmax = 4*dtavg;
    
    for i=1:N-1
        [~,dt_i_inds] = sample_indices(i, nx);
        lb(dt_i_inds) = dtmin;
        ub(dt_i_inds) = dtmax;
    end
    
    % set initial trajectory from the simulation
    z0 = zeros(nz,1);
    for i=1:N
        [x_i_inds,dt_i_inds] = sample_indices(i, nx);
        z0(x_i_inds) = x0(:,i)';
        if i < N
            z0(dt_i_inds) = dt_0(i)';
        end
    end

    options = optimoptions('fmincon','SpecifyObjectiveGradient',true,...
        'SpecifyConstraintGradient',true,'Display','iter');
    problem.objective = @trajectory_cost;


    problem.x0 = z0;
    problem.options = options;
    problem.nonlcon = @(z) all_constraints(z, N, nx);
    problem.solver = 'fmincon';
    problem.lb = lb;
    problem.ub = ub;
    z = fmincon(problem);
    x = zeros(4,N);
    t = zeros(1,N);
    for i=1:N
        [x_i_inds,dt_i_inds] = sample_indices(i, nx);
        x(:,i) = z(x_i_inds);
        if i < N
            t(i + 1) = t(i) + z(dt_i_inds);
        end
    end
    
end

function [g,dG] = trajectory_cost(z)
    % we're just finding behavior, rather than optimizing it, so we can
    % equivalently treat our cost as zero.
    g = 0;
    dG = zeros(numel(z),1);
end

function [c, ceq, dC, dCeq] = all_constraints(z, N, nx)
    % assemble continuous dynamics and hybrid jump constraints together
    [ceq_dyn, dCeq_dyn] = dynamics_constraints(z, N, nx);    
    [c_cyc, ceq_cyc, dC_cyc, dCeq_cyc] = cycle_constraints(z, N, nx);
    
    ceq = [ceq_dyn; ceq_cyc];
    dCeq = [dCeq_dyn; dCeq_cyc];
    
    c = c_cyc;
    dC = dC_cyc;
    
    dC = sparse(dC)';
    dCeq = sparse(dCeq)';
end