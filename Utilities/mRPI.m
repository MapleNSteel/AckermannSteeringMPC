close all

LineariseSystem
A1=double(subs(A, [v rho Lf Lr], [1.5 6.40001858964 1.2888 1.2884]));
B1=double(subs(B, [v rho Lf Lr], [1.5 6.40001858964 1.2888 1.2884]));
C1=double(subs(C, [v rho Lf Lr], [1.5 6.40001858964 1.2888 1.2884]));

ds=1e-1;

A2=eye(3)+A1*ds;
B2=B1*ds;

[K, S, e]=lqrd(A1, B1, diag([1, 1e-5, 1]), diag([1e-5 1e-5]), ds);

display(e);

As=A2-B2*K;

W=Polyhedron('A', [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1], 'b', [0.00401; 0.00401; 0.015; 0.015; 0.0; 0.0]);
temp=W;
for k=1:3
    temp=W+As*temp;
end
figure; temp.plot();
Z=temp;

vMin=1.0;
vMax=13.0;

sMin=-0.6;
sMax=0.6;

yeMin=-0.1;
yeMax=0.1;

psieMin=-pi/3;
psieMax=pi/3;

tMin=0.0043;
tMax=0.04;

U=Polyhedron('A', [1 0; -1 0; 0 1; 0 -1], 'b', [vMax; -vMin; sMax; -sMin]);
X=Polyhedron('A', [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1], 'b', [yeMax; -yeMin; psieMax; -psieMin; tMin; tMax]);

Ub=U-(-K*(Z));
Xb=X-Z;

figure;
Ub.plot();
figure;
Xb.plot();