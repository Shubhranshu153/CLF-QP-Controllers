function [M,C,T] = MCT(m,m_h,a,b,l,X)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

theta_ns=X(1,1);
theta_s=X(2,1);
theta_dot_ns=X(3,1);
theta_dot_s=X(4,1);
g = 9.81;

M=[m*b*b,-m*l*b*cos(theta_s-theta_ns);-m*l*b*cos(theta_s -theta_ns),m_h*l*l+m*(l*l+a*a)];
C=[0 m*l*b*theta_dot_s*sin(theta_s-theta_ns);-m*l*b*theta_dot_ns*sin(theta_s-theta_ns) 0];
T=[m*g*b*sin(theta_ns);-(m_h*l+m*(a+l))*g*sin(theta_s)];


% M=[m*b*b,-m*l*b*cos(x(2)-x(1));-m*l*b*cos(x(2) -x(1)),m_h*l*l+m*(l*l+a*a)];
% C=[0,m*l*b*x(4)*sin(x(2)-x(1));-m*l*b*x(3)*sin(x(2)-x(1)),0];
% T=[m*g*b*sin(x(1));-(m_h*l+m*(a+l))*g*sin(x(2))];

end

