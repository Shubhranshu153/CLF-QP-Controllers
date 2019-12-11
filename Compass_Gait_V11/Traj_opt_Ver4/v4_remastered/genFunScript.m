    %Script to generate functions
    syms d1 d2 d3 d4 d5 l1 l2 l3 l4 l5 I1 I2 I3 I4 I5 m1 m2 m3 m4 m5
    syms th1 th2 th3 th4 th5 th1dot th2dot th3dot th4dot th5dot g
    p=params;
    M = load('M.mat');
    C = load('C.mat');
    T = load('T.mat');

    Mtest = M.M;
    Ttest = T.T;
    Ctest = C.C;


    Msubbed =subs(Mtest, [d1, d2, d3, d4, d5, l1, l2, l3, l4, l5, I1, I2,...
            I3, I4, I5, m1, m2, m3, m4, m5],[p.d1, p.d2, p.d3, p.d4, p.d5,...
            p.l1, p.l2, p.l3, p.l4, p.l5, p.I1, p.I2, p.I3, p.I4, p.I5,...
            p.m1, p.m2, p.m3, p.m4, p.m5]);
    matlabFunction(Msubbed,'file','testMatrixM');


    Csubbed = subs(C.C, [d1, d2, d3, d4, d5, l1, l2, l3, l4, l5, I1, I2,...
            I3, I4, I5, m1, m2, m3, m4, m5],[p.d1, p.d2, p.d3, p.d4, p.d5,...
            p.l1, p.l2, p.l3, p.l4, p.l5, p.I1, p.I2, p.I3, p.I4, p.I5,...
            p.m1, p.m2, p.m3, p.m4, p.m5]);
    Csubbed = Csubbed; %needs to be a row vector
    matlabFunction(Csubbed,'file','testMatrixC');


    Tsubbed = subs(T.T, [d1, d2, d3, d4, d5, l1, l2, l3, l4, l5, I1, I2,...
            I3, I4, I5, m1, m2, m3, m4, m5,g],[p.d1, p.d2, p.d3, p.d4, p.d5,...
            p.l1, p.l2, p.l3, p.l4, p.l5, p.I1, p.I2, p.I3, p.I4, p.I5,...
            p.m1, p.m2, p.m3, p.m4, p.m5,p.g]);
     matlabFunction(Tsubbed,'file','testMatrixT');