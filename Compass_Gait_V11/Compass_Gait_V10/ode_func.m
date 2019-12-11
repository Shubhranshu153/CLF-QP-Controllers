
function dxdt=ode_func(t,x,x_oneCycle,kp, kd, Q, epsilon, B)
% global x_oneCycle t_oneCycle kp kd Q epsilon B
global V_keep tforv_keep

%       
        mySpline = spline(x_oneCycle(2,:),x_oneCycle([1,3,4],:));
        p=params();
        X=x;
        k = ppval(mySpline,X(2));
        yd = [k(1);X(2);k(2);k(3)];
        y_err= X - yd; %y and ydot
        eta = y_err;
     
        m_pos_err=p.m*1;
        mh_pos_err=p.m*1;
        a_pos_err=p.a*1;
        b_pos_err=p.b*1;
        
        
        m_neg_err=-p.m*0.5;
        mh_neg_err=-p.m*0.5;
        a_neg_err=-p.a*0.5;
        b_neg_err=-p.b*0.5;
        
      
        [M,C,T]=MCT(p.m,p.mh,p.a,p.b,p.a+p.b,x);
        [Mpred,Cpred,Tpred]=MCT(p.m+m_pos_err,p.mh+mh_pos_err,p.a+a_pos_err,p.b+b_pos_err,p.a+p.b+a_pos_err+b_pos_err,x);
        [M_neg_pred,C_neg_pred,T_neg_pred]=MCT(p.m+m_neg_err,p.mh+mh_neg_err,p.a+a_neg_err,p.b+b_neg_err,p.a+p.b+a_neg_err+b_neg_err,x);
        
         B = [1,0; 0 -1];
        
        % Calculate V and Vdot, also generate P, use current state
        [V_pos,V_pos_dot_f,V_pos_dot_g, P_pos]=Lyapunov(eta,kp,kd,Q,epsilon, Mpred,Cpred,Tpred,B,X);
         [V_neg,V_neg_dot_f,V_neg_dot_g, P_neg]=Lyapunov(eta,kp,kd,Q,epsilon, M_neg_pred,C_neg_pred,T_neg_pred,B,X);
        c3_pos = min(eig(Q))/max(eig(P_pos)); %grizzle torque saturation in bipedal robotic walking through contorl lyaponov function based quadratic programs 
         c3_neg= min(eig(Q))/max(eig(P_neg));
        CLF_pos_constraint = -((V_pos_dot_f)+(c3_pos/epsilon)*V_pos);
        CLF_neg_constraint = -((V_neg_dot_f)+(c3_neg/epsilon)*V_neg);
        mu = quadprog_qp(CLF_pos_constraint,V_pos_dot_g,CLF_neg_constraint,V_neg_dot_g);
        V_keep = [V_keep V_pos];
        tforv_keep = [tforv_keep t];
      
        dxdt = zeros(4,1);
        dxdt(1) = x(3)+ 0.0*sin(t);
        dxdt(2) = x(4) +0.0*sin(t);
        k =inv(M)*(-C*[x(3);x(4)] - T) + inv(M)*B*[mu(1);mu(2)];
        %k =inv(M)*B*U_optimal;
        dxdt(3) = k(1,1)+0.0*cos(t);
        dxdt(4) = k(2,1)+0.0*cos(t);

end