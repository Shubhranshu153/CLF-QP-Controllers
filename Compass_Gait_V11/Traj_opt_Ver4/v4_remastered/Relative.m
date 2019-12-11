% Matlab functiob to transfor absolute angles model to relative angle model 
% Relative angles are the angles connecting the extension of each link
% with the adjacent link
% The relative angles are shown in the attached biped.pdf as q0,q1,q2,q3,q4
% © 18/06/2017 Ibrahim Ali Mohammed Seleem ALL RIGHTS RESERVED
% If you have any question, don't hesitate to contact me 
% ibrahim.seleem@ejust.edu.eg
function [Dq,Hq,Gq]=Relative()

syms xb yb d1 d2 d3 d4 d5 l1 l2 l3 l4 l5 th1 th2 th3 th4 th5 th1dot th2dot th3dot th4dot th5dot
syms th1dotdot th2dotdot th3dotdot th4dotdot th5dotdot
syms I1 I2 I3 I4 I5 m1 m2 m3 m4 m5      %Inertia & masses
syms g      % gravitional acceleration
syms q0 q1 q2 q3 q4 q0dot q1dot q2dot q3dot q4dot q0dotdot q1dotdot q2dotdot q3dotdot q4dotdot

global M N G 


[M,N,G]=Generalized();

M=subs(M,[th1 th2 th3 th4 th5],[q0 (q0-q1) (q0-q1-q2) (-q0+q1+q2+q3) (-q0+q1+q2+q3-q4)]) ;

N=subs(N,[th1,th2,th3,th4,th5,th1dot,th2dot,th3dot,th4dot,th5dot],...
    [q0,(q0-q1),(q0-q1-q2),(-q0+q1+q2+q3),(-q0+q1+q2+q3-q4),...
    q0dot,(q0dot-q1dot),(q0dot-q1dot-q2dot),(-q0dot+q1dot+q2dot+q3dot),(-q0dot+q1dot+q2dot+q3dot-q4dot)]);

G=subs(G,[th1 th2 th3 th4 th5],[q0 (q0-q1) (q0-q1-q2) (-q0+q1+q2+q3) (-q0+q1+q2+q3-q4)]) ;


for j=1:5
        A1(j)=M(1,j)+M(2,j)+M(3,j)-M(4,j)-M(5,j);
        A2(j)=-M(2,j)-M(3,j)+M(4,j)+M(5,j);
        A3(j)=-M(3,j)+M(4,j)+M(5,j);
        A4(j)=M(4,j)+M(5,j);
        A5(j)=-M(5,j);
end

    % compute Inertia Matrix
        L11=A1(1)+A1(2)+A1(3)-A1(4)-A1(5); % D(1,1)
        L12=-A1(2)-A1(3)+A1(4)+A1(5); % D(1,2)
        L13=-A1(3)+A1(4)+A1(5); % D(1,3)
        L14=A1(4)+A1(5); % D(1,4)
        L15=-A1(5); % D(1,5)
       
        L21=A2(1)+A2(2)+A2(3)-A2(4)-A2(5); % D(2,1)
        L22=-A2(2)-A2(3)+A2(4)+A2(5); % D(2,2)
        L23=-A2(3)+A2(4)+A2(5); % D(2,3)
        L24=A2(4)+A2(5); % D(2,4)
        L25=-A2(5); % D(2,5)   

        L31=A3(1)+A3(2)+A3(3)-A3(4)-A3(5); % D(3,1)
        L32=-A3(2)-A3(3)+A3(4)+A3(5); % D(3,2)
        L33=-A3(3)+A3(4)+A3(5); % D(3,3)
        L34=A3(4)+A3(5); % D(3,4)
        L35=-A3(5); % D(3,5)        
        
        L41=A4(1)+A4(2)+A4(3)-A4(4)-A4(5);% D(4,1)
        L42=-A4(2)-A4(3)+A4(4)+A4(5); % D(4,2)
        L43=-A4(3)+A4(4)+A4(5); % D(4,3)
        L44=A4(4)+A4(5); % D(4,4)
        L45=-A4(5); % D(4,5)       
        
        L51=A5(1)+A5(2)+A5(3)-A5(4)-A5(5); % D(4,1)
        L52=-A5(2)-A5(3)+A5(4)+A5(5); % D(4,2)
        L53=-A5(3)+A5(4)+A5(5); % D(4,3)
        L54=A5(4)+A5(5); % D(4,4)
        L55=-A5(5); % D(4,5)   
        
        Dq=[L11,L12,L13,L14,L15;
            L21,L22,L23,L24,L25;
            L31,L32,L33,L34,L35;
            L41,L42,L43,L44,L45;
            L51,L52,L53,L54,L55];
 

    % compute cotilios matrix
        hq0=N(1)+N(2)+N(3)-N(4)-N(5);
        hq1=-N(2)-N(3)+N(4)+N(5);
        hq2=-N(3)+N(4)+N(5);
        hq3=N(4)+N(5);
        hq4=-N(5);
    
       [E1,F1]=coeffs(hq0,[q0dot q1dot q2dot q3dot q4dot]);
       [E2,F2]=coeffs(hq1,[q0dot q1dot q2dot q3dot q4dot]);
       [E3,F3]=coeffs(hq2,[q0dot q1dot q2dot q3dot q4dot]);
       [E4,F4]=coeffs(hq3,[q0dot q1dot q2dot q3dot q4dot]);
       [E5,F5]=coeffs(hq4,[q0dot q1dot q2dot q3dot q4dot]);
    
       size(F1);
       size(F2);
       size(F3);
       size(F4);
       size(F5);
    
       Hq11=E1(1)*q1dot+E1(2)*q2dot+E1(3)*q3dot+E1(4)*q4dot; %q0dot
       Hq12=E1(5)*q1dot+E1(6)*q2dot+E1(7)*q3dot+E1(8)*q4dot; %q1dot
       Hq13=E1(9)*q2dot+E1(10)*q3dot+E1(11)*q4dot;           %q2dot
       Hq14=E1(12)*q3dot+E1(13)*q4dot;                       %q3dot
       Hq15=E1(14)*q4dot;                                    %q4dot    
    
       Hq21=E2(1)*q0dot+E2(2)*q2dot+E2(3)*q3dot+E2(4)*q4dot; %q0dot
       Hq22=E2(5)*q2dot+E2(6)*q3dot+E2(7)*q4dot;             %q1dot
       Hq23=E2(8)*q2dot+E2(9)*q3dot+E2(10)*q4dot;            %q2dot
       Hq24=E2(11)*q3dot+E2(12)*q4dot;                       %q3dot
       Hq25=E2(13)*q4dot;                                    %q4dot
   
       Hq31=E3(1)*q0dot+E3(2)*q1dot+E3(3)*q4dot;             %q0dot
       Hq32=E3(4)*q1dot+E3(5)*q4dot;                         %q1dot
       Hq33=E3(6)*q4dot;                                     %q2dot
       Hq34=E3(7)*q4dot;                                     %q3dot
       Hq35=E3(8)*q4dot;                                     %q4dot
    
       Hq41=E4(1)*q0dot+E4(2)*q1dot+E4(3)*q4dot;             %q0dot
       Hq42=E4(4)*q1dot+E4(5)*q4dot;                         %q1dot
       Hq43=E4(6)*q4dot;                                     %q2dot
       Hq44=E4(7)*q4dot;                                     %q3dot
       Hq45=E4(8)*q4dot;                                     %q4dot
    
       Hq51=E5(1)*q0dot+E5(2)*q2dot+E5(3)*q3dot+E5(4)*q4dot; %q0dot
       Hq52=E5(5)*q1dot+E5(6)*q2dot+E5(7)*q3dot;             %q1dot
       Hq53=E5(8)*q2dot+E5(9)*q3dot;                         %q2dot
       Hq54=E5(10)*q3dot;                                    %q3dot
       Hq55=0;                                               %q4dot    
    
       Hq=[Hq11,Hq12,Hq13,Hq14,Hq15;
          Hq21,Hq22,Hq23,Hq24,Hq25;
          Hq31,Hq32,Hq33,Hq34,Hq35;
          Hq41,Hq42,Hq43,Hq44,Hq45;
          Hq51,Hq52,Hq53,Hq54,Hq55];
       
    
      
    % compute Gravitional vector         
       gq0=G(1)+G(2)+G(3)-G(4)-G(5);
       gq1=-G(2)-G(3)+G(4)+G(5);
       gq2=-G(3)+G(4)+G(5);
       gq3=G(4)+G(5);
       gq4=-G(5);
      
       Gq=[gq0;gq1;gq2;gq3;gq4];

      
end    
    
    
    
    
    
    