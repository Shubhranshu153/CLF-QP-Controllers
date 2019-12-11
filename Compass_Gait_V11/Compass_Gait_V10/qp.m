function [out] = qp(clf_constraint, V_dot_g)
% Copyright 2019, Gurobi Optimization, LLC
%
% This example formulates and solves the following simple QP model:
%  minimize
%      x^2 + x*y + y^2 + y*z + z^2 + 2 x
%  subject to
%      x + 2 y + 3 z >= 4
%      x +   y       >= 1
%      x, y, z non-negative
%
% It solves it once as a continuous model, and once as an integer
% model.
tic
%names = {'mu', 'etheta_sw', 'etheta_st', 'etheta_dot_sw','etheta_dot_st' }; % mu is the objective input values
names={'mu1','mu2'};
model.varnames = names;
Q= eye(2);%zeros(1,4)]; %cost matrix for mu
%R= [zeros(4,1),eye(4,4)]; %cost matrix
model.Q = sparse(Q); 
% model.A = sparse([V_dot_g;1,-1]);
model.A = sparse([2 -2]);%;1 -1]);%used to be 1
%model.obj = []; %0 guessed as f but not sure
%model.rhs = [clf_constraint];%;0]; %pass the value required
model.rhs = -500;
model.sense = '<'; %was >

gurobi_write(model, 'qp.lp');

results = gurobi(model);

for v=1:length(names)
    fprintf('%s %e\n', names{v}, results.x(v));
end

fprintf('Obj: %e\n', results.objval);

model.vtype = 'I';

results  = gurobi(model);

for v=1:length(names)
    fprintf('%s %e\n', names{v}, results.x(v));
end

fprintf('Obj: %e\n', results.objval);
out = results.objval;
toc
end
