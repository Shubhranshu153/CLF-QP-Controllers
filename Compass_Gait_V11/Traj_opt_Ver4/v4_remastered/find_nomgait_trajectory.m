function [z, Aeq, beq, lb, ub, z0,t,qforanim,Anim] = find_nomgait_trajectory()
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
addpath('S:\snopt7\matlab');
addpath('S:\snopt7\matlab\precompiled');
theta_0 = [0.363914;0.6;0;-0.1;-0.7];
  %  theta_0 = [-0.7;-0.1;0;0.6;0.363914];
%     theta_0 = [-0.4;-0.7;0;0.26;0.34];
%     theta_0 = [deg2rad(-30);deg2rad(-45);deg2rad(-5);deg2rad(30);deg2rad(30)];
    theta_f = theta_0([5;4;3;2;1]);
    step_time=1;
    theta_0_dot = [-1.5;
-0.61;
0.01;
-0.55;
-2.00]; 
    
    theta_f_dot = [-2.17;
-1.629;
1.028;
-1.39;
3.015];
    x_0 = [theta_0];
    x_f = [theta_f];
    N = 100;
    dt = 0.001;
    nx = 5*2;
    nu = 5;
    nt=1;

    % TODO: Add constraints to Aeq, beq to enforce starting at x_0 and
    % ending at x_f
    [x_0_inds, ~] = sample_indices(1, nx, nu,nt);
    [x_f_inds, ~] = sample_indices(N, nx, nu,nt);
    [x_m_inds, ~] = sample_indices(N/2, nx, nu,nt);
    dt_0 =step_time/N;
         
    Aeq = zeros(nx, N * (nx + nu +nt));
    beq = zeros(nx, 1);

    Aeq(1:(nx/2), x_0_inds(1:5)) = eye(nx/2);
    Aeq((1:nx/2) + nx/2, x_f_inds(1:5)) = eye(nx/2);
   % Aeq((1:nx/2)+nx,x_m_inds(1:5))=eye(nx/2);
  % beq = [x_0; x_f;x_mid];
   beq = [x_0; x_f];

    
    
    
    
    uBound = 1000; %bounds on u

    % TODO: Add bounding box constraints u \in [-M,M]^nu
    lb = -inf(N * (nx + nu),1);
    ub = inf(N * (nx + nu),1);
    for i=1:N
        [~,u_i_inds] = sample_indices(i, nx, nu,nt);
        lb(u_i_inds) = -uBound;
        ub(u_i_inds) = uBound;
    end
    
    % add the constraints on states
    qLow = (-pi);
    qUpp = (pi);
    dqLow = -10;
    dqUpp = 10;
    monotonic_velocity_low = -10;
    torso_angle_low=-0.18; % this works dont change torso angle limits
    torso_angle_high=0.18;
    
    dtavg = step_time/(N-1);
    dtmin = 0.25*dtavg;
    dtmax = 2*dtavg;
    for i=1:N
        [x_i_inds,~,dt_i_inds] = sample_indices(i, nx, nu,nt);
         lb(x_i_inds(1:5))=qLow;
         lb(x_i_inds(6:10))=dqLow;
         ub(x_i_inds(1:5))=qUpp;
         ub(x_i_inds(6:10))=dqUpp;
         lb(x_i_inds(3))=torso_angle_low;
         ub(x_i_inds(3))=torso_angle_high;
         lb(dt_i_inds) = dtmin;
         ub(dt_i_inds) = dtmax;
    end

    % TODO: make initial guess for z
    z0 = zeros(N * (nx + nu+ nt), 1);

    for i=1:N
        [x_i_inds,~,dt_i_inds] = sample_indices(i, nx, nu,nt);
        z0(x_i_inds(1:5)) = x_0+ ((i-1)/(N-1))*((x_f - x_0));
        z0(dt_i_inds) = dt_0;
    end
    
%     z0 = load('z0otherTry.mat');
%     z0=z0.xnew;
    
    %generate dynamics functions based on params
    %so they just take theta and thetadot as inputs
    genFunScript

    options = optimoptions('fmincon','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'Display','iter');
    problem.objective = @(z) trajectory_cost(z, N, nx, nu,nt, dt);


    problem.x0 = z0;
    problem.options = options;
    problem.nonlcon = @(z) all_constraints(z, N, nx, nu, dt);
    problem.solver = 'fmincon';
    problem.Aeq = Aeq;
    problem.beq = beq;
    problem.lb = lb;
    problem.ub = ub;

%     z = fmincon(problem);
    A = [];
    b = [];
    snscreen on;
    snprint('snopt_output');
    x0=z0;
z = snsolve(@trajectory_cost, x0, A, b, Aeq, beq, lb, ub, @all_constraints);
x1=z;
z = snsolve(@trajectory_cost, x1, A, b, Aeq, beq, lb, ub, @all_constraints);
% x1=z;
% z = snsolve(@trajectory_cost, x1, A, b, Aeq, beq, lb, ub, @all_constraints);
% % x2=z;
% % z = snsolve(@trajectory_cost, x2, A, b, Aeq, beq, lb, ub, @all_constraints);
% %     
    qforanim=[];
    t=[];
    t(1)=0;
    for i=1:N
        [x_i_inds,~,t_i_inds] = sample_indices(i, nx, nu,nt);
        temp = z(x_i_inds);
        qforanim=[qforanim [temp(1:5)]];
        if i>1
           t(i)=t(i-1)+z(t_i_inds);
        end
    end
%     param is a 1x1 struct containing: 
    param = getPhysicalParameters();
    param.stepLength = 0.5;
        param.stepTime = 0.7;
    Anim.speed = 0.5;
    Anim.plotFunc = @(t,qforanim)( drawRobot(qforanim,param) );
    Anim.verbose = true;
    animate(t,qforanim,Anim);
end

function [c, ceq, dC, dCeq] = all_constraints(z)%, N, nx, nu, dt)
    %Non-linear constraints
    %c -> <=0
    N = 100;
    dt = 0.0001;
    nx = 5*2;
    nu = 5;
    nt=1;
    [c,ceq, dCeq] = dynamics_constraints(z, N, nx, nu,nt);

    dC = zeros(N*(nx+nu+nt),4*(N-1)+1)';

    dc=
    dCeq = sparse(dCeq);
end