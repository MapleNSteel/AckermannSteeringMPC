syms Lr Lf v d psie ye xd yd psid psi
syms rho 

% 

beta=atan(Lr*tan(d)/(Lf+Lr));
xd = v*cos(psi+beta);
yd = v*sin(psi+beta);
psid = v*sin(beta)/Lr;