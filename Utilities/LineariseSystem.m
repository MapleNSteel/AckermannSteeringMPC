syms Lr Lf v d psie ye T xd yd psid psi deltaTime
syms rho 
syms dt

% Kinematic Bicycle

beta=atan(Lr*tan(d)/(Lf+Lr));
xd=v*cos(psi+beta);
yd=v*sin(psi+beta);
psid=v*sin(beta)/Lr;

% Spatial Reformulation

ds=rho*(v*cos(beta)*cos(psie)-v*sin(beta)*sin(psie))/(rho-ye);
dye=(v*sin(beta)*cos(psie)+v*cos(beta)*sin(psie))/ds;
dpsie=(psid/ds) - 1/rho;

% Deriving Matrices:

vec=[ye; psie];
f=[dye; dpsie];

du=atan(((Lr+Lf)/Lr)*tan(asin(Lr/rho)));

A=simplify(subs(jacobian(f, [ye psie]), [ye psie d], [0 0 du]));
B=simplify(subs(jacobian(f, [v d]), [ye psie d], [0 0 du]));
C=simplify(subs(f-B*[v; du], [ye psie d], [0 0 du]));
