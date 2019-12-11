function [g,dG] = trajectory_cost(z)%, N, nx, nu, dt)
%TRAJECTORY_COST(z) computes the cost and cost jacobian.
%   @param z: decision variable (column) vector containing the x_i and u_i
%   @param N: number of sample points; scalar
%   @param nx: dimension of state vector, x; scalar
%   @param nu: dimension of input vector, u; scalar
%   @param dt: \Delta t, the inter-sample interval duration; scalar

%   @output g: total accrued cost; scalar
%   @output dG: gradient of total accrued cost; nz by 1 vector
    N = 100;
    dt = 0.1;
    nx = 5*2;
    nu = 5;
    nt=1;
    g = 0;
    dG = zeros(N*(nx + nu+nt),1);
    
    for i=1:(N-1)
%         [~,u_i_inds,~] = sample_indices(i, nx, nu,nt);
%         [~,u_ip1_inds,~] = sample_indices(i + 1, nx, nu,nt);
%         g = g + dt*0.5*norm(z(u_i_inds))^2 + dt*0.5*norm(z(u_ip1_inds))^2;
%         dG(u_i_inds) = dG(u_i_inds) + dt*z(u_i_inds);
%         dG(u_ip1_inds) = dG(u_ip1_inds) + dt*z(u_ip1_inds);

    end

end