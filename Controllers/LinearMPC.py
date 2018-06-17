import numpy as np
import cvxopt

cvxopt.matrix_repr = cvxopt.printing.matrix_str_default
cvxopt.printing.options['dformat'] = '%.2f'
cvxopt.printing.options['width'] = -1
cvxopt.solvers.options['show_progress'] = False
#cvxopt.solvers.options['maxiters'] = 10
#cvxopt.solvers.options['abstol'] = 1e-10
#cvxopt.solvers.options['reltol'] = 1e-10
#cvxopt.solvers.options['feastol'] = 1e-10 

def getControl(A, B, C, x, r, g1, g2, h1, h2, stateLength, controlLength, N, Q, R, S, Cbar):

	A=cvxopt.matrix(A)
	B=cvxopt.matrix(B)
	C=cvxopt.matrix(C)

	x=cvxopt.matrix(x)
	r=cvxopt.matrix(r)

	if(not (g1==None or h1==None or g2==None or h2==None)):
		g1=cvxopt.matrix(g1)
		h1=cvxopt.matrix(h1)
		g2=cvxopt.matrix(g2)
		h2=cvxopt.matrix(h2)

	Q=cvxopt.matrix(Q)
	R=cvxopt.matrix(R)
	S=cvxopt.matrix(S)
	
	Cbar=cvxopt.matrix(Cbar)

	Rhat=cvxopt.spdiag([R for i in range(0,N)])
	Qhat=cvxopt.spdiag([C.trans()*Q*C if i<(N-1) else C.trans()*S*C for i in range(0,N)])

	b=[]
	temp=cvxopt.matrix(np.eye(stateLength))
	for i in range(0,N):
		b.append(temp)
		temp=temp+A*b[-1]

	T1=cvxopt.sparse(b)
	Chat=T1*Cbar

	That=cvxopt.spdiag([Q*C if i<(N-1) else C.trans()*S*C for i in range(0,N)])
	

	b=[]
	for i in range(0,N):
		a=[]
		for j in range(0,N):
			if(j<=i):
				a.append((cvxopt.matrix(np.linalg.matrix_power(A, i-j))*B).trans())
			else:
				a.append(cvxopt.matrix(np.zeros(B.size)).trans())
		b.append(a)

	Bhat=cvxopt.sparse(b).trans()
	Ahat=cvxopt.sparse([cvxopt.matrix(np.linalg.matrix_power(A, i+1)) for i in range(0,N)])

	#Final Matrices
	P1=(Bhat.trans()*Qhat*Bhat+Rhat)
	q=(x.trans()*Ahat.trans()*Qhat*Bhat-r.trans()*That*Bhat+Chat.trans()*Qhat*Bhat).trans()

	if(not (g1==None or h1==None or g2==None or h2==None)):
		G=cvxopt.sparse([g1, g2*Bhat])
		H=cvxopt.matrix(cvxopt.sparse([h1, h2-g2*Chat-g2*Ahat*x]), tc='d')
	else:
		G=None
		H=None
	
	sol=cvxopt.solvers.qp(P1,q,G,H)

	return sol['x'], Ahat*x+Bhat*sol['x']+Chat

