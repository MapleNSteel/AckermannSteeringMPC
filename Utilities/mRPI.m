close all

LineariseSystem
A1=double(subs(A, [rho Lf Lr], [1000 1.2888 1.2884]));
B1=double(subs(B, [rho Lf Lr], [1000 1.2888 1.2884]));
C1=double(subs(C, [rho Lf Lr], [1000 1.2888 1.2884]));

A2=eye(2)+A1*1e-1;
B2=B1*1e-1;

[K, S, e]=lqrd(A1, B1, diag([1, 1e-4]), diag([1e-2 1e-3]), 1e-1);

As=A2-B2*K;

W=Polyhedron('A', [1 0; -1 0; 0 1; 0 -1], 'b', [0.001; 0.001; 0.01; 0.01]);
temp=W;
for k=1:7
    temp=temp+As^k*W;
end
figure; temp.plot();
Z=temp;

vMin=1.0;
vMax=1.0;

sMin=-0.6;
sMax=0.6;

yeMin=-0.1;
yeMax=0.1;

psieMin=-pi/3;
psieMax=pi/3;

U=Polyhedron('A', [1 0; -1 0; 0 1; 0 -1], 'b', [vMax; -vMin; sMax; -sMin]);
X=Polyhedron('A', [1 0; -1 0; 0 1; 0 -1], 'b', [yeMax; -yeMin; psieMax; -psieMin]);

Ub=U-K*Z;
Xb=X-Z;

figure;
Ub.plot();
figure;
Xb.plot();