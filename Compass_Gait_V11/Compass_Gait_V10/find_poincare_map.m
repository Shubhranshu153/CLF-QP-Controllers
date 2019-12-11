function [P, M, eigen] = find_poincare_map(t, x, dy)
%FIND_POINCARE_MAP(x, t, delta_x) takes the limit cycle described by (x, t)
%and constructs an appoximate poincare section y_f = P * y_i via finite
%differencing.
%
%   @param t: row vector with indices corresponding to the time of the
%   i-th state sample; 1 by N vector
%   @param x: matrix with columns corresponding to x_i, the ith state
%   sample in the limit cycle; nx by N matrix
%   @param dy: perturbation magnitude for y_i; scalar
%
%   @output P: linearized poincare section, approxmate jacobian of y_f
%   w.r.t. y_i at y_i = 0; (nx-1) by (nx-1) matrix
%   @output M: matrix with columns that span the poincare section; nx by
%   (nx-1) matrix



    
 
%     x_star = x(:,end);
%     y_star=M'*x_star;
%     x_star_delx=M*inv(M'*M)*(y_star + [delta_yi,0 ,0; 0,delta_yj,0;0,0,delta_yk]);
%     y_star_dely=M'*x_star_delx;
%     dyi= (y_star_dely(1) -y_star(1))/delta_yi;
%     dyj= (y_star_dely(2) -y_star(2))/delta_yj;
%     dyk= (y_star_dely(3) -y_star(3))/delta_yk;
%     P=[dyi 0 0; 0 dyj 0; 0 0 dyk];

    A = [1 1 0 0];
    M = null(A);
%     M = zeros(4,3);
%     M(1,1)=1;
%     M(2,1)=1;
%     M(3,2)=1;
%     M(4,3)=1;
    
   
    xstar = x(:,1);
    
%     deltay = [dy,dy,dy]';
    deltay1 = [1;0;0]*dy;
    deltay2 = [0;1;0]*dy;
    deltay3 = [0;0;1]*dy;
    
    deltax1 = pinv(M')*deltay1;
    deltax2 = pinv(M')*deltay2;
    deltax3 = pinv(M')*deltay3;
    
%     deltax = pinv(M')*deltay;
%     tf = t(1);
    x_tot1= xstar+deltax1;
    x_tot2= xstar+deltax2;
    x_tot3= xstar+deltax3;
    
    [~, xnew1] = simulate_compassgait(x_tot1, 2);
    [~, xnew2] = simulate_compassgait(x_tot2, 2);
    [~, xnew3] = simulate_compassgait(x_tot3, 2);
    
    temp1 = xnew1{2};
    temp2 = xnew2{2};
    temp3 = xnew3{2};
    
    y1 = M'*temp1(:,1);
    y2 = M'*temp2(:,1);
    y3 = M'*temp3(:,1);
    
    ystar = M'*xstar;
    
    P1 = (y1-ystar)/dy;
    P2 = (y2-ystar)/dy;
    P3 = (y3-ystar)/dy;
    
    P = [P1,P2,P3];
    
%     P = ones(3);
%     P(1,1)=P1;
%     P(2,2)=P2;
%     P(3,3)=P3;
    
    eigen = eig(P);
    
    
end