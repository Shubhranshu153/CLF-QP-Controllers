function [P] = P_matrix(kp,kd,Q)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
A=A_matrix(kp,kd);
P=lyap(A,Q);
end

