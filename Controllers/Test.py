
from cvxopt import matrix, printing
from numpy import array
import numpy as np
import scipy
from numpy import pi
from numpy import tan,arctan,sin,cos,arctan2,sign,fmod,sqrt
from time import sleep
import math
import cvxopt
cvxopt.matrix_repr = printing.matrix_str_default
printing.options['dformat'] = '%.2f'
printing.options['width'] = -1
stateLength=2
controlLength=2
N=4

Q=cvxopt.matrix(np.array(np.diag([1, 0.01])))
C=cvxopt.matrix(np.eye(stateLength))
Ctilde=cvxopt.sparse([[C], [cvxopt.matrix(np.zeros((controlLength,controlLength)))]])
S=Q*2#Terminal Cost -x

b=[]
for i in range(0,N):
	a=[]
	if(i!=N-1):
		temp=Q*Ctilde
	else:
		temp=S*Ctilde
	for j in range(0,N):
		if(j==i):
			a.append(temp)
		else:
			a.append(cvxopt.matrix(np.zeros((stateLength+controlLength, stateLength))).trans())
	b.append(a)

That=cvxopt.sparse(b)

print(cvxopt.sparse(b))
