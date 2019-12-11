function [z, Aeq, beq, lb, ub, z0] = find_swingup_trajectory(x_0, x_f, N, dt)
%FIND_SWINGUP_TRAJECTORY(x_0, x_f, N, dt) executes a direct collocation
%optimization program to find an input sequence to drive the cartpole
%system from x_0 to x_f.
%
%   @param x_0: the state at the start of the trajectory; n_x by 1 vector
%   @param x_f: the state at the emd of the trajectory; n_x by 1 vector
%
%   @output z: decision variable vector containing the x_i and u_i
%   @output Aeq: matrix from linear constrant Aeq z = beq
%   @output beq: (column) vector from linear constrant Aeq z = beq
%   @output lb: lower bound of constraint lb <= z <= ub; n_z by 1 vector
%   @output ub: upper bound of constraint lb <= z <= ub; n_z by 1 vector
%   @output z0: initial guess for z; n_z by 1 vector
    nx = 4;
    nu = 1;

    % TODO: Add constraints to Aeq, beq to enforce starting at x_0 and
    % ending at x_f
    [x_0_inds, ~] = sample_indices(1, nx, nu);
    [x_f_inds, ~] = sample_indices(N, nx, nu);
    Aeq = zeros(2*nx, N * (nx + nu));
    beq = zeros(2*nx, 1);

    Aeq(1:nx, x_0_inds) = eye(nx);
    Aeq((1:nx) + nx, x_f_inds) = eye(nx);
    beq = [x_0; x_f];

    M = 50;

    % TODO: Add bounding box constraints u \in [-M,M]^nu
    lb = -inf(N * (nx + nu),1);
    ub = inf(N * (nx + nu),1);
    for i=1:N
        [~,u_i_inds] = sample_indices(i, nx, nu);
        lb(u_i_inds) = -M;
        ub(u_i_inds) = M;
    end

    % TODO: make initial guess for z
    z0 = zeros(N * (nx + nu), 1);

    for i=1:N
        [x_i_inds,~] = sample_indices(i, nx, nu);
        z0(x_i_inds) = x_0 + ((i-1)/(N-1))*(x_f - x_0);
    end

    options = optimoptions('fmincon','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'Display','iter');
    problem.objective = @(z) trajectory_cost(z, N, nx, nu, dt);


    problem.x0 = z0;
    problem.options = options;
    problem.nonlcon = @(z) all_constraints(z, N, nx, nu, dt);
    problem.solver = 'fmincon';
    problem.Aeq = Aeq;
    problem.beq = beq;
    problem.lb = lb;
    problem.ub = ub;

    z = fmincon(problem);
end

function [c, ceq, dC, dCeq] = all_constraints(z, N, nx, nu, dt)

    [ceq, dCeq] = dynamics_constraints(z, N, nx, nu, dt);

    c = zeros(0,1);
    dC = zeros(0,numel(z));

    dC = sparse(dC)';
    dCeq = sparse(dCeq)';
end