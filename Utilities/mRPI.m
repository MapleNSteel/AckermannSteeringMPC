close all

LineariseSystem
du=atan(((Lr+Lf)/Lr)*tan(asin(Lr/rho)));

A1=double(subs(A, [v d rho Lf Lr], [.15 du 6.40001858964 0.3247 0.2753]));
B1=double(subs(B, [v d rho Lf Lr], [.15 du 6.40001858964 0.3247 0.2753]));
C1=double(subs(C, [v d rho Lf Lr], [.15 du 6.40001858964 0.3247 0.2753]));

ds=1e-1;

A2=eye(2)+A1*ds;
B2=B1*ds;

[K, S, e]=lqrd(A1, B1, diag([1, 1e-5]), diag([1e-5 1e-5]), ds);

display(e);

As=A2-B2*K;

W=Polyhedron('A', [1 0; -1 0; 0 1; 0 -1], 'b', [0.005; 0.005; 0.015; 0.015]);
temp=W;
for k=1:3
    temp=W+As*temp;
end
figure; temp.plot();
Z=temp;

vMin=.10;
vMax=.20;

sMin=-0.6;
sMax=0.6;

yeMin=-0.1;
yeMax=0.1;

psieMin=-pi/6;
psieMax=pi/6;

U=Polyhedron('A', [1 0; -1 0; 0 1; 0 -1], 'b', [vMax; -vMin; sMax; -sMin]);
X=Polyhedron('A', [1 0; -1 0; 0 1; 0 -1], 'b', [yeMax; -yeMin; psieMax; -psieMin]);

Ub=U-(-K*(Z));
Xb=X-Z;

figure;
Ub.plot();
figure;
Xb.plot();