function [mu] = quadprog_qp(clf_pos_constraint,V_pos_dot_g,clf_neg_constraint,V_neg_dot_g)

tic
%names = {'mu', 'etheta_sw', 'etheta_st', 'etheta_dot_sw','etheta_dot_st' }; % mu is the objective input values
p1=1000;
p2=1000;

H=[1 0 0 0;
   0 1 0 0;
   0 0 p1 0;
   0 0 0 p2];
%zeros(1,4)]; %cost matrix for mu
f = [0 0 0 0];
A = [V_pos_dot_g(1) V_pos_dot_g(2) -1 0;
    V_neg_dot_g(1) V_neg_dot_g(2)  0 -1;
     1,0,0 0;
     -1,0,0 0;
      0,1,0 0;
      0,-1,0 0;
      0 0  1 0;
      0 0 -1 0;
      0 0 0  1;
      0 0 0 -1];
b = [clf_pos_constraint;
     clf_neg_constraint;
     100;
     100;
     100;
     100;
      2;
      0;
      2;
      0];
Aeq = [0 0 0 0]; % because B already has a -ve
%Aeq=[0 0];
beq = 0;

[mu,fval,exitflag,output,lambda] = ...
   quadprog(H,f,A,b,Aeq,beq);

fprintf('mu1 : %.2f \nmu2 : %.2f \n', mu(1),mu(2));
fprintf('fval : %.2f \n', fval);
fprintf('exitflag : %.2f \n', exitflag);
% fprintf('output : %.2f \n', fval);
% fprintf('lambda : %.2f \n', fval);



end