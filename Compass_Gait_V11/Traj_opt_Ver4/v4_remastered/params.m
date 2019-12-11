function p = params

%Parameters for the link lenghs
%d's are the distance from the base of the link to the com
%l's are the total link lengths
%distances in meters
p.d1 = 0.168;
p.l1 = 0.32;
p.d2 = 0.18;
p.l2 = 0.32;
p.d3 = 0.25;
p.l3 = 0.5;
p.d4 = 0.18;
p.l4 = 0.32;
p.d5 = 0.168;
p.l5 = 0.32;

%ground slope
p.gamma = deg2rad(3);

%gravity
p.g = 9.81;

%link masses - upper leg 5x lower leg (see paper)
%mass in kg
p.m1 = 3.3;
p.m2 = 8.1;
p.m3 = 37.9;
p.m4 = 8.1;
p.m5 = 3.3;

%link moment of intertias
p.I1 = 0.029;
p.I2 = 0.078;
p.I3 = 0.79;
p.I4 = 0.078;
p.I5 = 0.029;

% p.a = 0.5;
% p.b = 0.5;
% p.c = 0.5;
% p.d = 0.5;
% p.e = 0.5;
% p.mh = 10;
% p.m = 5;
% p.g = 9.8;
% p.gamma = 3*pi/180;
% p.gamma = 4*pi/180;

end

