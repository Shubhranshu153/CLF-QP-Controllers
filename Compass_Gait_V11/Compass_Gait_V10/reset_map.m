function [R,dR] = reset_map(xm)
%RESET_MAP(xm) returns the post-impact state as a function of the
%pre-impact state, as well as the jacobian of this function.
%
%   @param xm: pre-impact state
%
%   @output R: post-impact state
%   @output dR: jacobian of xp with respect to xm


dth_st = xm(4,:);
dth_sw = xm(3,:);
th_st = xm(2,:);
th_sw = xm(1,:);
t2 = dth_sw.*1.3e+1;
t3 = -th_sw;
t4 = dth_st.*6.5e+1;
t5 = t3+th_st;
t6 = cos(t5);
t7 = sin(t5);
t8 = t6.^2;
t9 = dth_sw.*t6.*4.0;
t14 = dth_st.*t6.*2.6e+2;
t10 = t8.*4.0;
t11 = -t9;
t13 = t8.*2.4e+1;
t15 = dth_st.*t8.*2.0e+1;
t16 = -t14;
t12 = dth_sw.*t10;
t17 = t10-1.3e+1;
t18 = t13-1.3e+1;
t21 = t4+t11+t15;
t19 = 1.0./t17;
R = [th_st;th_sw;dth_sw.*t6.*t19.*2.0-dth_st.*t18.*t19;dth_sw.*t19-dth_st.*t6.*t19.*1.0e+1];
if nargout > 1
    t22 = t2+t12+t16;
    t20 = t19.^2;
    t23 = t7.*t20.*t21.*2.0;
    t24 = t7.*t20.*t22.*2.0;
    dR = reshape([0.0,1.0,-t24,t23,1.0,0.0,t24,-t23,0.0,0.0,t6.*t19.*2.0,t19,0.0,0.0,-t18.*t19,t6.*t19.*-1.0e+1],[4,4]);
end
