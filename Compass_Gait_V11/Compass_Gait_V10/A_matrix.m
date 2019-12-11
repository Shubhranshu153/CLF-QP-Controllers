function [A] = A_matrix(kp,kd)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

A=zeros(4,4);
A(1,3)=1;
A(2,4)=1;
A(3,1) = -kp;
A(4,2) = -kp;
A(3,3) = -kd;
A(4,4) = -kd;
end

