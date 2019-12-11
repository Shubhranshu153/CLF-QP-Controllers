% Matlab function to calculate the dynamic model depending on absolute angles
%absoulte angles are the angles between each link and the vertical axis
% The absolute angles are shown in the attached biped.pdf as theta1,theta2,theta3,theta4,theta5
% © 18/06/2017 Ibrahim Ali Mohammed Seleem ALL RIGHTS RESERVED
% If you have any question, don't hesitate to contact me 
% ibrahim.seleem@ejust.edu.eg
function [M,N,G]=Generalized()
clc
% close all
% clear all

syms xb yb d1 d2 d3 d4 d5 l1 l2 l3 l4 l5 th1 th2 th3 th4 th5 th1dot th2dot th3dot th4dot th5dot
syms I1 I2 I3 I4 I5 m1 m2 m3 m4 m5      %Inertia & masses
syms g      % gravitional acceleration
syms th1dotdot th2dotdot th3dotdot th4dotdot th5dotdot

% The position of the free end
xe=xb+l1*sin(th1)+l2*sin(th2)+l4*sin(th4)+l5*sin(th5);
ye=yb+l1*cos(th1)+l2*cos(th2)-l4*cos(th4)-l5*cos(th5);

% velocity of the free end
xedot=diff(xe,th1)*th1dot+diff(xe,th2)*th2dot+diff(xe,th3)*th3dot+diff(xe,th4)*th4dot+diff(xe,th5)*th5dot;
yedot=diff(ye,th1)*th1dot+diff(ye,th2)*th2dot+diff(ye,th3)*th3dot+diff(ye,th4)*th4dot+diff(ye,th5)*th5dot;
ve=[xedot;yedot];

%%
% The position of Center of mass of each link
% link1, shank1
xc1=d1*sin(th1);
yc1=d1*cos(th1);

%link2, thigh1
xc2=l1*sin(th1)+d2*sin(th2);
yc2=l1*cos(th1)+d2*cos(th2);

%link3, trunk
xc3=l1*sin(th1)+l2*sin(th2)+d3*sin(th3);
yc3=l1*cos(th1)+l2*cos(th2)+d3*cos(th3);

%link4, thigh2
xc4=l1*sin(th1)+l2*sin(th2)+(l4-d4)*sin(th4);
yc4=l1*cos(th1)+l2*cos(th2)-(l4-d4)*cos(th4);

% link5, shank2
xc5=l1*sin(th1)+l2*sin(th2)+l4*sin(th4)+(l5-d5)*sin(th5);
yc5=l1*cos(th1)+l2*cos(th2)-l4*cos(th4)-(l5-d5)*cos(th5);

%%
% The linear velocity of the center of the mass of each link
% link1, shank1
xc1dot=diff(xc1,th1)*th1dot;
yc1dot=diff(yc1,th1)*th1dot;
vc1=[xc1dot;yc1dot];

%link2, thigh1
xc2dot=diff(xc2,th1)*th1dot+diff(xc2,th2)*th2dot;
yc2dot=diff(yc2,th1)*th1dot+diff(yc2,th2)*th2dot;
vc2=[xc2dot;yc2dot];

%link3, trunk
xc3dot=diff(xc3,th1)*th1dot+diff(xc3,th2)*th2dot+diff(xc3,th3)*th3dot;
yc3dot=diff(yc3,th1)*th1dot+diff(yc3,th2)*th2dot+diff(yc3,th3)*th3dot;
vc3=[xc3dot;yc3dot];

%link4, thigh2
xc4dot=diff(xc4,th1)*th1dot+diff(xc4,th2)*th2dot+diff(xc4,th3)*th3dot+diff(xc4,th4)*th4dot;
yc4dot=diff(yc4,th1)*th1dot+diff(yc4,th2)*th2dot+diff(yc4,th3)*th3dot+diff(yc4,th4)*th4dot;
vc4=[xc4dot;yc4dot];

% link5, shank2
xc5dot=diff(xc5,th1)*th1dot+diff(xc5,th2)*th2dot+diff(xc5,th3)*th3dot+diff(xc5,th4)*th4dot+diff(xc5,th5)*th5dot;
yc5dot=diff(yc5,th1)*th1dot+diff(yc5,th2)*th2dot+diff(yc5,th3)*th3dot+diff(yc5,th4)*th4dot++diff(yc5,th5)*th5dot;
vc5=[xc5dot;yc5dot];

%%
% Equation of motion
Mt=[m1,m2,m3,m4,m5];
THDOTt=[th1dot,th2dot,th3dot,th4dot,th5dot];
% THDOTDOTt=[th1dotdot,th2dotdot,th3dotdot,th4dotdot,th5dotdot];
It=[I1,I2,I3,I4,I5];
YCt=[yc1,yc2,yc3,yc4,yc5];
VCt=[(vc1(1).^2+vc1(2).^2),(vc2(1).^2+vc2(2).^2),(vc3(1).^2+vc3(2).^2),(vc4(1).^2+vc4(2).^2),(vc5(1).^2+vc5(2).^2)];

for i=1:5
    k(i)=simplify(0.5*Mt(i)*VCt(i)+0.5*It(i)*THDOTt(i).^2);
    p(i)=simplify(Mt(i)*g*YCt(i));
end
K=sum(k);
P=sum(p);
L=K-P;

Equations=Lagrange(L,[th1 th1dot th1dotdot th2 th2dot th2dotdot th3 th3dot th3dotdot th4 th4dot th4dotdot th5 th5dot th5dotdot]);

T1=Equations(1);
T2=Equations(2);
T3=Equations(3);
T4=Equations(4);
T5=Equations(5);

% T=collect(T1,[th1dotdot th2dotdot th3dotdot th4dotdot th5dotdot th1dot.^2 th2dot.^2 th3dot.^2 th4dot.^2 th5dot.^2]);
[D1,M1]=coeffs(T1,[th1dotdot th2dotdot th3dotdot th4dotdot th5dotdot th1dot th2dot th3dot th4dot th5dot]);
[D2,M2]=coeffs(T2,[th1dotdot th2dotdot th3dotdot th4dotdot th5dotdot th1dot th2dot th3dot th4dot th5dot]);
[D3,M3]=coeffs(T3,[th1dotdot th2dotdot th3dotdot th4dotdot th5dotdot th1dot th2dot th3dot th4dot th5dot]);
[D4,M4]=coeffs(T4,[th1dotdot th2dotdot th3dotdot th4dotdot th5dotdot th1dot th2dot th3dot th4dot th5dot]);
[D5,M5]=coeffs(T5,[th1dotdot th2dotdot th3dotdot th4dotdot th5dotdot th1dot th2dot th3dot th4dot th5dot]);

D1=simplify(D1.');
D2=simplify(D2.');
D3=simplify(D3.');
D4=simplify(D4.');
D5=simplify(D5.');

%%
% inertia Matrix
M=[ D1(1),D1(2),D1(3),D1(4),D1(5);
    D2(1),D2(2),D2(3),D2(4),D2(5);
    D3(1),D3(2),D3(3),0,0;
    D4(1),D4(2),0,D4(3),D4(4);
    D5(1),D5(2),0,D5(3),D5(4)];

% corilios matrix
N=[0*th1dot,D1(6)*th2dot,D1(7)*th3dot,D1(8)*th4dot,D1(9)*th5dot;
   D2(6)*th1dot,0*th2dot,D2(7)*th3dot,D2(8)*th4dot,D2(9)*th5dot;
   D3(4)*th1dot,D3(5)*th2dot,0*th3dot,0*th4dot,0*th5dot;
   D4(5)*th1dot,D4(6)*th2dot,0*th3dot,0*th4dot,D4(7)*th5dot;
   D5(5)*th1dot,D5(6)*th2dot,0*th3dot,D5(7)*th4dot,0*th5dot]*THDOTt.';
% Gravitional matrix
G=[D1(end);D2(end);D3(end);D4(end);D5(end)];

end