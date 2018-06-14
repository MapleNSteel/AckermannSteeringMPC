close all

LineariseSystem
A1=double(subs(A, [v rho Lf Lr], [1.5 6.40001858964 1.2888 1.2884]));
B1=double(subs(B, [v rho Lf Lr], [1.5 6.40001858964 1.2888 1.2884]));
C1=double(subs(C, [v rho Lf Lr], [1.5 6.40001858964 1.2888 1.2884]));

<<<<<<< HEAD
ds=1e-1;

A2=eye(3)+A1*ds;
B2=B1*ds;

[K, S, e]=lqrd(A1, B1, diag([1, 1e-5, 1]), diag([1e-5 1e-5]), ds);
=======
A2=eye(3)+A1*1e-1;
B2=B1*1e-1;

[K, S, e]=lqrd(A1, B1, diag([1e2, 1e-1, 1e-5]), diag([1e-3 1e-5]), 1e-1);
>>>>>>> 8d94c0442f34af4a66b86ab4e04dd1f76759ad75

display(e);

As=A2-B2*K;

<<<<<<< HEAD
W=Polyhedron('A', [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1], 'b', [0.00401; 0.00401; 0.015; 0.015; 0.0; 0.0]);
temp=W;
for k=1:3
=======
W=Polyhedron('A', [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1], 'b', [0.000401; 0.000401; 0.0015; 0.0015; 0.0; 0.0]);
temp=W;
for k=1:5
>>>>>>> 8d94c0442f34af4a66b86ab4e04dd1f76759ad75
    temp=W+As*temp;
end
figure; temp.plot();
Z=temp;

vMin=1.0;
<<<<<<< HEAD
vMax=13.0;
=======
vMax=2.0;
>>>>>>> 8d94c0442f34af4a66b86ab4e04dd1f76759ad75

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

<<<<<<< HEAD
Ub=U-(-K*(Z));
=======
Ub=U-(-K*(X+Z));
>>>>>>> 8d94c0442f34af4a66b86ab4e04dd1f76759ad75
Xb=X-Z;

figure;
Ub.plot();
figure;
Xb.plot();