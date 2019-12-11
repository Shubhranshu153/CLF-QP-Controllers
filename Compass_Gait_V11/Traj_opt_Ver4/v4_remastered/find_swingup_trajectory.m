function [z, Aeq, beq, lb, ub, z0] = find_swingup_trajectory()
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
    theta_0 = [-0.4;-0.7;0;0.26;0.34];
    theta_f = theta_0([5;4;3;2;1]);
    step_time=1;
    theta_0_dot = (theta_f-theta_0)/step_time; 
    theta_f_dot = theta_0_dot;
    x_0 = [theta_0;theta_0_dot];
    x_f = [theta_f;theta_f_dot];
    N = 50;
    dt = 0.1;
    nx = 5*2;
    nu = 5;

    % TODO: Add constraints to Aeq, beq to enforce starting at x_0 and
    % ending at x_f
    [x_0_inds, ~] = sample_indices(1, nx, nu);
    [x_f_inds, ~] = sample_indices(N, nx, nu);
    Aeq = zeros(2*nx, N * (nx + nu));
    beq = zeros(2*nx, 1);

    Aeq(1:nx, x_0_inds) = eye(nx);
    Aeq((1:nx) + nx, x_f_inds) = eye(nx);
    beq = [x_0; x_f];

    uBound = 1000; %bounds on u

    % TODO: Add bounding box constraints u \in [-M,M]^nu
    lb = -inf(N * (nx + nu),1);
    ub = inf(N * (nx + nu),1);
    for i=1:N
        [~,u_i_inds] = sample_indices(i, nx, nu);
        lb(u_i_inds) = -uBound;
        ub(u_i_inds) = uBound;
    end

    % TODO: make initial guess for z
    z0 = zeros(N * (nx + nu), 1);

    for i=1:N
        [x_i_inds,~] = sample_indices(i, nx, nu);
        z0(x_i_inds) = x_0 + ((i-1)/(N-1))*(x_f - x_0);
    end
    
%     z0 = load('z0otherTry.mat');
%     z0=z0.xnew;
    
    %generate dynamics functions based on params
    %so they just take theta and thetadot as inputs
    genFunScript

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
    
    qforanim=[];
    for i=1:N
        [x_i_inds,~] = sample_indices(i, nx, nu);
        temp = z(x_i_inds);
        qforanim=[qforanim temp(1:5)];
    end
%     param is a 1x1 struct containing: 
    param = getPhysicalParameters();
    param.stepLength = 0.5;
    param.stepTime = 0.7;
    Anim.speed = 0.25;
    t = 0.1:0.1:5;
    Anim.plotFunc = @(t,qforanim)( drawRobot(qforanim,param) );
    Anim.verbose = true;
    animate(t,qforanim,Anim);
end

function [c, ceq, dC, dCeq] = all_constraints(z, N, nx, nu, dt)

    [ceq, dCeq] = dynamics_constraints(z, N, nx, nu, dt);

    c = zeros(0,1);
    dC = zeros(0,numel(z));

    dC = sparse(dC)';
    dCeq = sparse(dCeq)';
end