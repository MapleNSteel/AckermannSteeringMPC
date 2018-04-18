#Doesnae work rae
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
cvxopt.solvers.options['show_progress'] = False
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter
from CurvilinearCoordinates import *

running=True
elapsedTime=0
deltaTime=10./1000.
deltaSigma=0

xSigmaPrev=0


Lr=1.2888
Lf=1.2884

L=3.3138

controlInput=cvxopt.matrix(np.array([[0.5],[0]]))

stateLength=2
controlLength=2

Q=cvxopt.matrix(np.eye(stateLength))*100#Running Cost - x
R=cvxopt.matrix(np.eye(controlLength))*1#Running Cost - u
P=cvxopt.matrix(np.eye(stateLength))*1000#Terminal Cost -x

N=10 #Window length
T=0

def forwardDifference(N, l):
	D=np.eye(N*l)
	
	for i in range(l, N*l):
		D[i, i-l]=-1

	return D

def jacobianF(u,rho,beta,psi):

	global deltaSigma

	v=u[0]
	d=u[1]

	return np.array([[-sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))/(rho*cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))), 2/(cos(2*psi + 2*arctan((Lr*sin(d))/(cos(d)*(Lf + Lr)))) + 1)],[-tan(d)/(rho*cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))*(Lf + Lr)*((Lr**2*tan(d)**2)/(Lf + Lr)**2 + 1)**(1/2)), (sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))*tan(d))/(cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2*(Lf + Lr)*((Lr**2*tan(d)**2)/(Lf + Lr)**2 + 1)**(1/2))]])*deltaSigma

def jacobianH(u,rho,beta,psi):

	global deltaSigma

	v=u[0]
	d=u[1]

	return np.array([[0, (Lr*(tan(d)**2 + 1)*(Lf + Lr))/(cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2*(Lr**2*tan(d)**2 + Lf**2 + Lr**2 + 2*Lf*Lr))], [0, ((tan(d)**2 + 1)*(Lf*cos(psi + arctan((Lr*tan(d))/(Lf + Lr))) + Lr*cos(psi + arctan((Lr*tan(d))/(Lf + Lr))) + Lr*sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))*tan(d)))/(cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))**2*(Lf + Lr)**2*((Lr**2*tan(d)**2)/(Lf + Lr)**2 + 1)**(3/2))]])

def exit_gracefully(signum, frame):

	running = False
	
	original_sigint = signal.getsignal(signal.SIGINT)
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)

	try:
		if input("\nReally quit? (y/n)> ").lower().startswith('y'):
			time.sleep(1)
			sys.exit(1)

	except KeyboardInterrupt:
		print("Ok ok, quitting")
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

	global startTime, velocity, accum, ySigmaPrev, Kp, Ki, Kd, pubySigma, CC, CC1, CC2, xSigmaPrev, T, elapsedTime, controlInput, deltaSigma

	cc=CC
	[phi, xSigma, ySigma, psiSigma, ds]=traj(x, y, velocity, psi, beta, cc)
	deltaSigma=xSigma-xSigmaPrev
	xSigmaPrev=xSigma
	pubySigma.publish(ySigma)
	
	x=cvxopt.matrix(np.array([[ySigma[0]], [psiSigma[0]]]))

	Qs=cvxopt.spdiag([Q if i<(N-1) else P for i in range(0,N)])
	Rs=cvxopt.spdiag([R for i in range(0,N)])

	B=cvxopt.matrix(jacobianH(cvxopt.matrix(controlInput), cc.rho(phi), beta, psi))
	C=cvxopt.matrix(np.eye(stateLength))
	A=cvxopt.matrix(jacobianF(cvxopt.matrix(controlInput), cc.rho(phi), beta, psi))

	G=cvxopt.sparse([C*(A**i) for i in range(0,N)])
	H=cvxopt.sparse([cvxopt.sparse([[C*(A**j)*B if j<=i else cvxopt.matrix(np.zeros(np.shape(B)))] for j in range(0,N)]) for i in range(0,N)])
	F=cvxopt.sparse([C*(A**i)*B for i in range(0,N)])

	#Final Matrice
	D=cvxopt.matrix(forwardDifference(N, controlLength))
	Y=Rs+H.trans()*Qs.trans()*H

	P1=2*Y
	q=H.trans()*Qs*G*x
	G1=cvxopt.sparse([cvxopt.matrix([[0,1], [0,-1]]).trans() for i in range(0,N)])
	c1=cvxopt.matrix(np.array([[pi/3],[pi/3]]))
	h1=cvxopt.matrix(cvxopt.sparse([c1 for i in range(0,N)]), (N*2,1))

	sol=cvxopt.solvers.qp(P1,q)#,G1,h1)

	controlInput=controlInput+sol['x'][0:2]

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

	dtSigma=sqrt(v.x*cos(psiSigma)-v.y*sin(psiSigma))/(1-ySigma/CC.rho(phi))

	return [phi, xSigma, ySigma, psiSigma, dtSigma]		

def callbackOdom(msg):

	global pubThrottle, pubSteering, position, velocity, orientation, angularVelocity, elapsedTime

	elapsedTime+=deltaTime
	Pose=msg.pose.pose
	Twist=msg.twist.twist

	position=Pose.position
	orientation=Pose.orientation

	velocity=Twist.linear
	angularVelocity=Twist.angular	

	theta, alpha, beta, psi=getAngles(position, orientation, velocity, angularVelocity)

	controlInput=control(position.x,position.y,psi,psi-theta)

	print(controlInput)

	pubThrottle.publish(controlInput[0])
	pubSteering.publish(controlInput[1])

	#print("psi"+str(psi))
	#print("Alpha"+str(alpha))
	#print("Beta"+str(beta))
	#print("theta"+str(theta))
	#print()

def main():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime, startTime, pubThrottle, pubSteering, pubySigma, CC, CC1, CC2
	
	rospy.init_node('Data')
	startTime=time.time()

	rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)
	pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=10)
	pubSteering = rospy.Publisher('/ackermann/Steering', Float32, queue_size=10)

	pubySigma = rospy.Publisher('/ackermann/ySigma', Float32, queue_size=10)

	X=lambda t: 5*cos(t)-5
	Y=lambda t: 5*sin(t)
	tangent=lambda t: np.array([-5*sin(t), 5*cos(t)])

	rho=lambda t: 5

	CC=CurvilinearCoordinates(X,Y,tangent,rho)

	X1=lambda t: 5*(1-0.1*cos(3*t))*cos(t)-4.5
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
		pass
if __name__=="__main__":
	main()
