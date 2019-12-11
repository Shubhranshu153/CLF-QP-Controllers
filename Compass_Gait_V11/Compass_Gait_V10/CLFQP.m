function [T_all, X_all, X_list,T_list, V_list,ustar_list,t_oneCycle, x_oneCycle, Eta,tforv_keep,V_keep] = CLFQP()%kp,kd,Q,epsilon)
%X state matrix [thetaswing thetastance thetadotswing thetadotstance]'
%kp and kd as scalars (they get repeated)
%Q PD matrix of size xlengh x xlength
%epsilon <1 and >0
%B: input ( [1 0; 0 -1] for compass)
% global x_oneCycle t_oneCycle kp kd Q epsilon B
global V_keep tforv_keep

kp =5; %Used to calculate P for the lyapunov function
kd = 0.5; %Used to calculate P for the lyapunov function
Q = 1*eye(4); %Q of Lyapunov Function
epsilon = 0.5; %Epsilon is used in Lyapunov to ensure exponential convergence


%Generate nominal trajectory
[t_oneCycle, x_oneCycle] = find_limit_cycle;
%[t_desTraj, x_desTraj] = simulate_compassgait(x_oneCycle(:,1), 20*(t_oneCycle(1)+t_oneCycle(end)) );
% x(2) is monotonoically decreasing
 %takes q2, gives q1 q3 q4



B = [1,0; 0 -1];
X_start = x_oneCycle(:,1);
X = X_start+[0.1;0.05/2.5;0.1;0.1]*2.5;
X_list = X; %list of actual states to plot later
T_list = 0; %list of actual times of actual states to plot later
V_list = [];
Eta=[];
ustar_list = [];

tf=5;

[T_all,X_all] = simulate_compassgait1(X, tf,x_oneCycle, t_oneCycle, kp, kd, Q, epsilon, B); %To check if the default cycle is stable remove the 1 at the end of gait. 


   
%% displays
figure
plot(tforv_keep,V_keep);
title('V')
plot_compassgait_trajectory(T_all, X_all)
title('walking?')
%plot
%add input name of plot, save v plot with gif
end



% %for each step
% [U_optimal] = CLFQP(X,kp,kd,Q,epsilon,c3,B,U)
% 
% %use u_optimal to calculate next position
% %plot next position
% 
% 
% end

