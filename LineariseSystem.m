syms Lr Lf v d psie ye xd yd psid psi
syms rho 

% Kinematic Bicycle

beta=atan(Lr*tan(d)/(Lf+Lr));
xd=v*cos(psi+beta);
yd=v*sin(psi+beta);
psid=v*sin(beta)/Lr;

% Spatial Reformulation

ds=rho*(xd*cos(psie)-yd*sin(psie))/(rho-ye);
dpsie=(psid/ds) - 1/rho;
dye=(xd*sin(psie)+yd*cos(psie))/ds;