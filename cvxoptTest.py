import cvxopt
import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos

Lr=1.2888
Lf=1.2884

deltaTime = 1./100
elapsedTime=0

N=2 #Window length
def jacobianF(x,u):

	beta=arctan((Lr/(Lf+Lr))*tan(u[1]))

	return np.array(np.eye(3))+np.array([[0,0,-u[0]*sin(x[2]+beta)*(u[0]/Lr)*sin(beta)],[0,0,u[0]*cos(x[2]+beta)*(u[0]/Lr)*sin(beta)],[0,0,0]])

def jacobianH(x,u):
	beta=arctan((Lr/(Lf+Lr))*tan(u[1]))
	dbeta=(Lr*(Lf+Lr))/((Lr*sin(u[1])**2+((Lf+Lr)*cos(u[1]))**2)

	return np.array([[cos(x[2]+beta), -u[0]*sin(x[2]+beta)*dbeta],[sin(x[2]+beta), 1],[sin(beta)/Lr, 0]])

#Elementary Matrices
N=20

x=cvxopt.matrix(np.array([[1], [0], [0]]))
u=cvxopt.matrix(np.array([[1], [0]]))

C=cvxopt.matrix(np.eye(3))
Q=cvxopt.matrix(np.eye(3))#Running Cost - x
A=cvxopt.matrix(jacobianF(x, u))
B=cvxopt.matrix(jacobianH(x))
P=cvxopt.matrix(np.eye(2))#Terminal Cost -x
Qs=cvxopt.spdiag([Q for i in range(0,N)])
G=cvxopt.sparse([C*(A**i) for i in range(0,N)])
H=cvxopt.sparse([cvxopt.sparse([[C*(A**j)*B if j<=i else cvxopt.matrix(np.zeros(np.shape(B)))] for j in range(0,N)]) for i in range(0,N)])

#Final Matrices
Y=H.trans()*Qs.trans()*H
PHI=cvxopt.sparse([P,G.trans()*Qs*G])
F=2*G.trans()*Qs*H

P=2*Y
q=(x.trans()*F).trans()
print(P)
print(q)
sol=cvxopt.solvers.qp(P,q)
print(sol['x'])
