import cvxopt
import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos

Lr=1.2888
Lf=1.2884

deltaTime = 1./100
elapsedTime=0

#Elementary Matrices
N=10

x=cvxopt.matrix(np.array([[1], [0], [0]]))
u=cvxopt.matrix(np.array([[1], [0]]))

Q=cvxopt.matrix(np.eye(3))#Running Cost - x
R=cvxopt.matrix(np.eye(2))#Running Cost - u
P=cvxopt.matrix(np.eye(3))#Terminal Cost -x

N=2 #Window length
def jacobianF(x,u):

	beta=arctan((Lr/(Lf+Lr))*tan(u[1]))

	return np.array(np.eye(3))+np.array([[0,0,-u[0]*sin(x[2]+beta)*(u[0]/Lr)*sin(beta)],[0,0,u[0]*cos(x[2]+beta)*(u[0]/Lr)*sin(beta)],[0,0,0]])

def jacobianH(x,u):
	beta=arctan((Lr/(Lf+Lr))*tan(u[1]))
	dbeta=(Lr*(Lf+Lr))/(Lr*sin(u[1])**2+((Lf+Lr)*cos(u[1]))**2)

	return np.array([[cos(x[2]+beta), -u[0]*sin(x[2]+beta)*dbeta],[sin(x[2]+beta), u[0]*cos(x[2]+beta)*dbeta],[sin(beta)/Lr, 0]])

def getControl():
	
	global Q, R, P

	Qs=cvxopt.spdiag([Q if i<(N-1) else P for i in range(0,N)])
	Rs=cvxopt.spdiag([R for i in range(0,N)])

	B=cvxopt.matrix(jacobianH(x,u))
	C=cvxopt.matrix(np.eye(3))
	A=cvxopt.matrix(jacobianF(x, u))

	G=cvxopt.sparse([C*(A**i) for i in range(0,N)])
	H=cvxopt.sparse([cvxopt.sparse([[C*(A**j)*B if j<=i else cvxopt.matrix(np.zeros(np.shape(B)))] for j in range(0,N)]) for i in range(0,N)])

	#Final Matrice
	Y=Rs+H.trans()*Qs.trans()*H
	PHI=cvxopt.sparse([G.trans()*Qs*G])
	F=2*G.trans()*Qs*H

	P=2*Y
	q=(x.trans()*F).trans()
	sol=cvxopt.solvers.qp(P,q)

	return np.array(sol['x'])

def main():
	u=getControl()[0:2]
	print(u)

if __name__=="__main__":
	main()
