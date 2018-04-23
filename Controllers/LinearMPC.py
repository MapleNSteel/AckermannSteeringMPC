import matplotlib.pyplot as plt
import numpy as np
import scipy
from numpy import pi
from numpy import tan,arctan,sin,cos,arctan2,sign,fmod,sqrt
from time import sleep
import math
import signal
import time
import sys
import rospy
import cvxopt
cvxopt.matrix_repr = cvxopt.printing.matrix_str_default
cvxopt.printing.options['dformat'] = '%.2f'
cvxopt.printing.options['width'] = -1
cvxopt.solvers.options['show_progress'] = False
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter
from CurvilinearCoordinates import *

import muaompc
import numpy
import system

running=True
elapsedTime=0
deltaTime=10./1000.
deltaSigma=0

xSigmaPrev=0


Lr=1.2888
Lf=1.2884

L=3.3138

controlInput=cvxopt.matrix(np.array([[0],[0]]))

stateLength=2
controlLength=2

Q=cvxopt.matrix(np.array(np.diag([1, 0.01])))#Running Cost - x
R=cvxopt.matrix(np.array(np.diag([1e-4, 1e-4])))#Running Cost - u
S=Q#Terminal Cost -x

N=5 #Window length
T=0

vmin=0.8
vmax=1.0

smin=-0.5
smax=0.5

ready=False

def jacobianF(u,rho,psi, ds):

	global deltaSigma

	v=u[0]
	d=u[1]

	return np.eye(stateLength)+np.array([[-sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))/(rho*cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))), sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2/cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2 + v*cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))],[-tan(d)/(rho*cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))*(Lf + Lr)*((Lr**2*tan(d)**2)/(Lf + Lr)**2 + 1)**(1/2)), (sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))*tan(d))/(cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2*(Lf + Lr)*((Lr**2*tan(d)**2)/(Lf + Lr)**2 + 1)**(1/2))]])*ds*deltaTime

def jacobianH(u,rho,psi ,ds):

	global deltaSigma

	v=u[0]
	d=u[1]

	return np.array([[0, (Lr*(tan(d)**2 + 1)*(rho*cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2 + rho*sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2)*(Lf + Lr))/(rho*cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2*(Lr**2*tan(d)**2 + Lf**2 + Lr**2 + 2*Lf*Lr))], [0, ((tan(d)**2 + 1)*(Lf*cos(psi + arctan((Lr*tan(d))/(Lf + Lr))) + Lr*cos(psi + arctan((Lr*tan(d))/(Lf + Lr))) + Lr*sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))*tan(d)))/(cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2*(Lf + Lr)**2*((Lr**2*tan(d)**2)/(Lf + Lr)**2 + 1)**(3/2))]])*ds*deltaTime

def exit_gracefully(signum, frame):

	running = False
	
	original_sigint = signal.getsignal(signal.SIGINT)
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)
	sys.exit(1)
	
	# restore the exit gracefully handler here	
	signal.signal(signal.SIGINT, exit_gracefully)

def getAngles(position, orientation, velocity, angularVelocity):
	angles=transforms3d.euler.quat2euler(np.array([orientation.w,orientation.x,orientation.y,orientation.z]))
	rot=np.array(transforms3d.euler.euler2mat(angles[0],angles[1],angles[2]))

	psi=np.arctan2(velocity.y,velocity.x)#Slip

	alpha=np.arctan2(rot[0,1], rot[0,2])
	beta=np.arctan2(rot[0,0], rot[0,2])
	theta=np.arctan2(-rot[2,1], -rot[2,0])

	return psi, alpha, beta, theta

def control(x, y, psi, beta):

	global startTime, velocity, accum, ySigmaPrev, Kp, Ki, Kd, pubySigma, CC, CC1, CC2, xSigmaPrev, elapsedTime, controlInput, deltaSigma, N, stateLength, controlLength, Lf, Lr, T, Q, R, S, vmin, vmax, smin, smax

	cc=CC
	[phi, xSigma, ySigma, psiSigma, ds]=traj(x, y, velocity, psi, beta, cc)
	deltaSigma=xSigma-xSigmaPrev
	xSigmaPrev=xSigma
	pubySigma.publish(ySigma)
	
	x=cvxopt.matrix(np.array([[ySigma[0]], [psiSigma[0]], [controlInput[0]], [controlInput[1]]]))
	r=cvxopt.sparse([cvxopt.matrix(np.array([[0.0], [0.0]])) for i in range(0,N)])

	B=cvxopt.matrix(jacobianH(cvxopt.matrix(controlInput), cc.rho(phi), psi ,ds))
	C=cvxopt.matrix(np.eye(stateLength))
	A=cvxopt.matrix(jacobianF(cvxopt.matrix(controlInput), cc.rho(phi), psi, ds))

	Atilde=cvxopt.matrix(cvxopt.sparse([[A, cvxopt.matrix(np.zeros(np.shape(B.trans())))],[B, cvxopt.matrix(np.eye(controlLength))]]))
	Btilde=cvxopt.sparse([[B,cvxopt.matrix(np.eye(controlLength))]])
	Ctilde=cvxopt.sparse([[C], [cvxopt.matrix(np.zeros((controlLength,controlLength)))]])

	Rhat=cvxopt.spdiag([R for i in range(0,N)])
	Qhat=cvxopt.spdiag([Ctilde.trans()*Q*Ctilde if i<(N-1) else Ctilde.trans()*S*Ctilde for i in range(0,N)])

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
	

	b=[]
	for i in range(0,N):
		a=[]
		for j in range(0,N):
			if(j<=i):
				a.append((cvxopt.matrix(numpy.linalg.matrix_power(Atilde, N-j))*Btilde).trans())
			else:
				a.append(cvxopt.matrix(np.zeros(Btilde.size)).trans())
		b.append(a)

	Bhat=cvxopt.sparse(b).trans()
	Ahat=cvxopt.sparse([cvxopt.matrix(numpy.linalg.matrix_power(Atilde, i+1)) for i in range(0,N)])

	#Final Matrices
	P1=(Bhat.trans()*Qhat*Bhat+Rhat)/2
	xAdj=cvxopt.sparse([[x.trans()], [r.trans()]])
	F=cvxopt.sparse([Ahat.trans()*Qhat*Bhat,-That*Bhat])
	q=cvxopt.matrix(xAdj*F).trans()
	#print(q)
	
	g=cvxopt.matrix(np.array([[0, 0, 1, 0], [0, 0, -1, 0], [0, 0, 0, 1], [0, 0, 0, -1]]), tc='d')
	h=cvxopt.matrix(np.array([[vmax], [-vmin], [smax], [-smin]]), tc='d')

	G=cvxopt.spdiag([g for i in range(0,N)])*Bhat
	H=cvxopt.sparse([h for i in range(0,N)])-cvxopt.spdiag([g for i in range(0,N)])*Ahat*x

	#print("cvxopt.spdiag([g for i in range(0,N)])")
	#print(cvxopt.spdiag([g for i in range(0,N)]))
	#print("Bhat")
	#print(Bhat)
	#print("G")
	#print(G)

	sol=cvxopt.solvers.qp(P1,q,G,H)
	controlInput=controlInput+sol['x'][0:2]
	print("phi:"+str(phi)+"    xSigma:"+str(xSigma)+"    ySigma:"+str(ySigma)+"    psiSigma:"+str(psiSigma))

	return np.array(controlInput)

def traj(x, y, v, psi, beta, CC):
	
	CC.setCoordinates(x,y)
	phi=CC.getCoordinates()

	xt=CC.X(phi)
	yt=CC.Y(phi)
	tangent=CC.tangent(phi)
	psit=arctan2(tangent[1], tangent[0])
	normal=np.array([tangent[1],-tangent[0]])

	xSigma=scipy.integrate.quad(lambda x: np.sqrt(CC.tangent(x)[0]**2+CC.tangent(x)[1]**2), 0, phi)[0]
	ySigma=cos(psit)*(y-yt) - sin(psit)*(x-xt)
	psiSigma=psi-psit

	dtSigma=(v.x*cos(psiSigma)-v.y*sin(psiSigma))/(1-ySigma/CC.rho(phi))

	return [phi, xSigma, ySigma, psiSigma, dtSigma]		

def callbackOdom(msg):

	global pubThrottle, pubSteering, position, velocity, orientation, angularVelocity, elapsedTime, ready

	elapsedTime+=deltaTime
	Pose=msg.pose.pose
	Twist=msg.twist.twist

	position=Pose.position
	orientation=Pose.orientation

	velocity=Twist.linear
	angularVelocity=Twist.angular

	#print("psi"+str(psi))
	#print("Alpha"+str(alpha))
	#print("Beta"+str(beta))
	#print("theta"+str(theta))
	#print()

	ready=True

def sendControls():
	
	global pubThrottle, pubSteering, position, velocity, orientation, angularVelocity, elapsedTime

	if(ready):

		theta, alpha, beta, psi=getAngles(position, orientation, velocity, angularVelocity)

		controlInput=control(position.x,position.y,psi,psi-theta)
		controlInput[1]=np.arctan2(sin(controlInput[1]), cos(controlInput[1]))

		print(controlInput)

		pubThrottle.publish(controlInput[0])
		pubSteering.publish(controlInput[1])
	else:
		return

def main():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime, startTime, pubThrottle, pubSteering, pubySigma, CC, CC1, CC2
	
	rospy.init_node('Data')
	startTime=time.time()

	rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)
	pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=1)
	pubSteering = rospy.Publisher('/ackermann/Steering', Float32, queue_size=1)

	pubySigma = rospy.Publisher('/ackermann/ySigma', Float32, queue_size=10)

	X=lambda t: 10*cos(t)-10
	Y=lambda t: 10*sin(t)
	tangent=lambda t: np.array([-sin(t), cos(t)])

	rho=lambda t: 10

	CC=CurvilinearCoordinates(X,Y,tangent,rho)

	X1=lambda t: 5*(1-0.1*cos(3*t))*cos(t)-4.0
	Y1=lambda t: 5*(1-0.1*cos(3*t))*sin(t)
	tangent1=lambda t: np.array([5*(0.1*3*sin(3*t))*cos(t)-5*(1-0.1*cos(3*t))*sin(t), 5*(0.1*3*sin(3*t))*sin(t)+5*(0.1*cos(3*t))*cos(t)])
	rho1= lambda t: 1/((abs((cos(2*t) + 4*cos(4*t) - 5*cos(t))/(abs(cos(4*t) - cos(2*t)/2 - 5*cos(t))**2 + abs(sin(2*t)/2 + sin(4*t) - 5*sin(t))**2)**(1/2) - ((2*sign(sin(2*t)/2 + sin(4*t) - 5*sin(t))*abs(sin(2*t)/2 + sin(4*t) - 5*sin(t))*(cos(2*t) + 4*cos(4*t) - 5*cos(t)) - 2*sign(cos(2*t)/2 - cos(4*t) + 5*cos(t))*abs(cos(4*t) - cos(2*t)/2 - 5*cos(t))*(sin(2*t) - 4*sin(4*t) + 5*sin(t)))*(sin(2*t)/2 + sin(4*t) - 5*sin(t)))/(2*(abs(cos(4*t) - cos(2*t)/2 - 5*cos(t))**2 + abs(sin(2*t)/2 + sin(4*t) - 5*sin(t))**2)**(3/2)))**2 + abs((10*sin(t) + 36*cos(t)*sin(t) - 64*cos(t)**3*sin(t))/(2*(abs(cos(4*t) - cos(2*t)/2 - 5*cos(t))**2 + abs(sin(2*t)/2 + sin(4*t) - 5*sin(t))**2)**(1/2)) + ((2*sign(sin(2*t)/2 + sin(4*t) - 5*sin(t))*abs(sin(2*t)/2 + sin(4*t) - 5*sin(t))*(cos(2*t) + 4*cos(4*t) - 5*cos(t)) - 2*sign(cos(2*t)/2 - cos(4*t) + 5*cos(t))*abs(cos(4*t) - cos(2*t)/2 - 5*cos(t))*(sin(2*t) - 4*sin(4*t) + 5*sin(t)))*(10*cos(t) + 18*cos(t)**2 - 16*cos(t)**4 - 3))/(4*(abs(cos(4*t) - cos(2*t)/2 - 5*cos(t))**2 + abs(sin(2*t)/2 + sin(4*t) - 5*sin(t))**2)**(3/2)))**2)**(1/2)/(abs(8*cos(t)**4 - 9*cos(t)**2 - 5*cos(t) + 3/2)**2 + abs(sin(2*t)/2 + sin(4*t) - 5*sin(t))**2)**(1/2))

	CC1=CurvilinearCoordinates(X1,Y1,tangent1,rho1)

	X2=lambda t: 0.1*(.05*t**2+0.15*t**3)
	Y2=lambda t: t
	tangent2=lambda t: np.array([0.1*(0.05*2*t+0.15*3*t**2), 1])
	rho2= lambda t: 1/((400*(abs((9*t + 1)*(4*t**2 - 36*t**3*sign(t*(9*t + 2))**2 - 81*t**4*sign(t*(9*t + 2))**2 - 4*t**2*sign(t*(9*t + 2))**2 + 36*t**3 + 81*t**4 + 40000*sign(t*(9*t + 2))**2))**2 + 40000*abs(t*(9*t + 2))**2*abs(sign(t*(9*t + 2))*(9*t + 1))**2*abs(sign(t*(9*t + 2)))**4)**(1/2))/(abs(sign(t*(9*t + 2)))**2*(abs(t*(9*t + 2))**2 + 40000)**2))
			
	CC2=CurvilinearCoordinates(X2,Y2,tangent1,rho2)

	global desiredSteeringAngle, desiredSpeed, position, rotation, velocity, angularVelocity
	signal.signal(signal.SIGINT, exit_gracefully)

	while(running):
		sendControls()
if __name__=="__main__":
	main()
