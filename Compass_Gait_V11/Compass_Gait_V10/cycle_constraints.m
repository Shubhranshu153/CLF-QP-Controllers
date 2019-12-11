function [c, ceq, dC, dCeq] = cycle_constraints(z, N, nx)
%CYCLE_CONSTRAINTS(z, N, nx) fomulates the constraints 1) x_1,...,x_{N-1}
%do not trigger jumps; 2) x_N triggers a jump; and 3) x_N jumps to x_1
%through through reset_map(x_N) as ceq(z) = 0 and c(z) <= 0.
%
%   @param z: decision variable (column) vector containing the state
%   samples and timestams [x_1; dt_1; ...; dt_{N-1}; x_N];
%   @param N: number of sample points; scalar
%   @param nx: dimension of state vector, x; scalar
%
%   @output c: inequality constraint column vector c(z) <= 0
%   @output ceq: equality constraint column vector ceq(z) = 0
%   @output dC: jacobian of c w.r.t. z; dim(c) by dim(z) matrix
%   @output dCeq: jacobian of ceq w.r.t. z; dim(ceq) by dim(z) matrix
   
    ceq = zeros(nx + 1, 1);
    xN = z((N-1)*nx+N-1+1:end);
    [R,dR] = reset_map(xN);
    ceq(1:nx)=z(1:nx)-R;  %x1-R(xN)=0
    [phi,dPhi] = guard(xN);
    ceq(nx+1) = phi;
    
    dCeq = zeros(nx + 1, numel(z));
    dCeq(nx+1,(N-1)*nx+N-1+1:end)= dPhi;
    for ii =1:nx
        for jj=1:nx
            if(ii==jj)
                dCeq(ii,jj)=1;
            end
            dCeq(ii,(N-1)*nx+N-1+jj) = -dR(ii,jj);
            
        end
    end
    
    c = zeros(N-1, 1);
    dC = zeros(N-1, numel(z));
    for ii = 1:(N-1)
        [phi,dPhi] = guard(z((ii-1)*nx+ii:(ii-1)*nx+ii+nx));
        c(ii)= -1*phi;
        dC(ii,nx*(ii-1)+ii:nx*(ii)+ii-1) = -dPhi';
    end
    

end

