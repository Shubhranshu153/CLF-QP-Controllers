function [phi,dPhi] = guard(x)
%GUARD(x) is the collision guard condition phi(x) > 0; a collision is
%triggered when guard(x) returns zero.
%
%   @param x: (possible) pre-impact state; nx by 1 vector.
%
%   @output phi: guard value; no impact is triggered if phi > 0; scalar
%   @output dPhi: jacobian of guard w.r.t. x; 1 by nx vector

    p = params();
    gamma = p.gamma;
    phi_1 = -x(1);
    phi_2 = x(1) + x(2) + 2*gamma;
    dPhi_1 = [-1 0 0 0]';
    dPhi_2 = [1 1 0 0]';

    if phi_1 > phi_2
        phi = phi_1;
        dPhi = dPhi_1;
    else
        phi = phi_2;
        dPhi = dPhi_2;
    end

end

