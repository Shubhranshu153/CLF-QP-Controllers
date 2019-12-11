function [h_i,dH_i] = dynamics_constraint_with_derivative(x_i, u_i, x_ip1, u_ip1, dt)
%DYNAMICS_CONSTRAINT_WITH_DERIVATIVE(x_i, u_i, x_ip1, u_ip1, dt) returns
%and computes the gradient of the vector constraint asssociated with
%dynamics_constraint(x_i, u_i, x_ip1, u_ip1, dt).
%
%   @param x_i: the state at the start of the interval; nx by 1 vector
%   @param u_i: the input at the start of the interval; nu by 1 vector
%   @param x_ip1: the state at the end of the interval; nx by 1 vector
%   @param u_ip1: the input at the end of the interval; nu by 1 vector
%   @param dt: \Delta t, the duration of the interval; scalar
%
%   @output h_i: constraint value from dynamics_constraint; nx by 1 vector
%   @output dH_i: jacobian of h_i w.r.t. [x_i; u_i; x_ip1; u_ip1]; nx by
%   (2nx + 2nu) matrix

    h_i = evaluate_dynamics_constraint(x_i, u_i, x_ip1, u_ip1, dt);
    if nargout > 1
      % use numerical derivatives to compute dH
      % dH = [dh/dx0 dh/du0 dh/dx1 dh/du1]
      % where the partial derivatives are written (dh/dx0)_ij = dh_i/dx0_j
      delta = 1e-8;
      dH_i = zeros(numel(x_i), 2*(numel(x_i)+numel(u_i)));
      for j=1:numel(x_i)
          dx = zeros(numel(x_i),1);
          dx(j) = delta;
          dHx_i_j = evaluate_dynamics_constraint(x_i + dx, u_i, x_ip1, u_ip1, dt) - h_i;
          dHx_ip1_j = evaluate_dynamics_constraint(x_i, u_i, x_ip1 + dx, u_ip1, dt) - h_i;
          dH_i(:,j) = dHx_i_j/delta;
          dH_i(:,j + numel(x_i) + numel(u_i)) = dHx_ip1_j/delta;
      end

      for j=1:numel(u_i)
          du = zeros(numel(u_i),1);
          du(j) = delta;
          dHu_i_j = evaluate_dynamics_constraint(x_i, u_i + du, x_ip1, u_ip1, dt) - h_i;
          dHu_ip1_j = evaluate_dynamics_constraint(x_i, u_i, x_ip1, u_ip1 + du, dt) - h_i;
          dH_i(:,j + numel(x_i)) = dHu_i_j/delta;
          dH_i(:,j + numel(x_i) + numel(u_i) + numel(x_ip1)) = dHu_ip1_j/delta;
      end
    end
end


function h_i = evaluate_dynamics_constraint(x_i, u_i, x_ip1, u_ip1, dt)
%EVALUATE_DYNAMICS_CONSTRAINT(x_i, u_i, x_ip1, u_ip1, dt) computes the
%   vector contstraint h_i(x_i, u_i, x_ip1, u_ip1) = 0 from Problem 2(c)
%
%   @param x_i: the state at the start of the interval; nx by 1 vector
%   @param u_i: the input at the start of the interval; nu by 1 vector
%   @param x_ip1: the state at the end of the interval; nx by 1 vector
%   @param u_ip1: the input at the end of the interval; nu by 1 vector
%   @param dt: \Delta t, the duration of the interval; scalar
%
%   @output h_i: quantity derived in Problem 2(c); nx by 1 vector

f_i = f(x_i, u_i);
f_ip1 = f(x_ip1, u_ip1);

s_i_mid = 0.5 * (x_i + x_ip1) - (dt / 8) * (f_ip1 - f_i);

s_i_dot_mid = (1.5 / dt) * (x_ip1 - x_i) - 0.25 * (f_i + f_ip1);

u_mid = 0.5*(u_i + u_ip1);

h_i = s_i_dot_mid - f(s_i_mid, u_mid);

end