function [V,V_dot_f,V_dot_g, P, f, g] = Lyapunov(eta,kp,kd,Q,epsilon, M,C,T,B,X)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
P=P_matrix(kp,kd,Q);
E = eye(4,4);
E(1,1) = 1/epsilon;
E(2,2) = 1/epsilon;
Peps = E*P*E;
V=eta'*Peps*eta;





syms del_sw del_st del_sw_dot del_st_dot

X_sym=[del_sw; del_st; del_sw_dot; del_st_dot];
V_sym = X_sym'*Peps*X_sym;
V_grad=jacobian(V_sym,X_sym);
V_grad= subs(V_grad, del_sw,eta(1,1));
V_grad=subs(V_grad, del_st,eta(2,1));
V_grad=subs(V_grad, del_sw_dot,eta(3,1));
V_grad=subs(V_grad, del_st_dot,eta(4,1));



f=[eta(3);
    eta(4);
    -inv(M)*(C*[eta(3); eta(4)] + T)];

g=[0,0;
   0,0;
   inv(M)*B];

V_dot_f = double(V_grad*f);
V_dot_g=double(V_grad*g);

end

