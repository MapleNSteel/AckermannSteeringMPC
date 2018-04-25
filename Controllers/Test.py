
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

d=[]
c=[]

vmin=0.8
vmax=1.0

smin=-0.5
smax=0.5


g=cvxopt.matrix(np.array([[0, 0, 1, 0], [0, 0, -1, 0], [0, 0, 0, 1], [0, 0, 0, -1]]), tc='d')
h=cvxopt.sparse([cvxopt.matrix(np.array([[vmax], [-vmin], [smax], [-smin]]), tc='d') for i in range(0,N)])

for i in range(0,N):
	a=[]
	b=[]
	for j in range(0,N):
		if(j==i):
			a.append(g)
			b.append(h)
		else:
			a.append(cvxopt.matrix(np.zeros(np.shape(g))))
	c.append(a)

g=cvxopt.sparse(c)

print(g)
print(h)
