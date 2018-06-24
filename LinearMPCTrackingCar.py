import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg
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
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter
from Utilities.CurvilinearCoordinates import *
from Controllers.LinearMPCTracking import *

running=True
timeDuration=0
startTime=time.time()
T=0

deltaTime=10./1000.
deltaSigma=0

xSigmaPrev=0

maxtimeDuration=0
mintimeDuration=1e2

Lr=1.2888
Lf=1.2884

controlInput=cvxopt.matrix(np.array([[1],[0]]))

stateLength=2
controlLength=2

Q=cvxopt.matrix(np.array(np.diag([1, 1e-3])))#Running Cost - x
R=cvxopt.matrix(np.array(np.diag([1e-5, 1e-5])))#Running Cost - u
S=cvxopt.matrix(np.array(np.diag([1, 1e-5])))#TerMinal Cost -x

K=np.zeros((stateLength, controlLength))

N=10 #Window length

vMin=-1.0
vMax=1.0

sMin=-0.6
sMax=0.6

yeMin=-0.1
yeMax=0.1

psieMin=-pi/3
psieMax=pi/3

# Robust Values; with ye, psie noise ranges - 1e-3, 1e-2
vMinRobust=1.0019
vMaxRobust=1.9811

sMinRobust=-0.5791
sMaxRobust=0.5791

yeMinRobust=-0.2468
yeMaxRobust=0.2468

psieMinRobust=-0.5396
psieMaxRobust=0.5396

g1=cvxopt.matrix(np.array([
   [ 1,    0],
   [-1,    0],
   [ 0,    1],
   [ 0,   -1]]), tc='d')
h1=cvxopt.sparse([cvxopt.matrix(np.array([
   [ 0.5],
   [-0.5],
   [ 0.7],
   [-0.7]]), tc='d') for i in range(0,N)])
g2=cvxopt.matrix(np.array([[1, 0, 0, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, -1, 0], [0, 0, 0, 1], [0, 0, 0, -1]]), tc='d')
h2=cvxopt.sparse([cvxopt.matrix(np.array([
   [yeMaxRobust], 
   [-yeMinRobust], 
   [psieMaxRobust], 
   [-psieMinRobust], 
   [ vMaxRobust],
   [-vMinRobust],
   [ sMaxRobust],
   [-sMinRobust]]), tc='d') for i in range(0,N)])

c=[]
#Sparsifying non-square matrix
for i in range(0,N):
	a=[]
	for j in range(0,N):
		if(j==i):
			a.append(g1)
		else:
			a.append(cvxopt.matrix(np.zeros(np.shape(g1))))
	c.append(a)

g1=cvxopt.sparse(c)

c=[]
for i in range(0,N):
	a=[]
	for j in range(0,N):
		if(j==i):
			a.append(g2)
		else:
			a.append(cvxopt.matrix(np.zeros(np.shape(g2))))
	c.append(a)

g2=cvxopt.sparse(c)

ready=False

def f(x, u, rho,psi, ds):

	v=u[0]
	d=u[1]

	return np.array([[sin(psi + arctan((Lr*tan(d))/(Lf + Lr)))/cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))], [tan(d)/(cos(psi + arctan((Lr*tan(d))/(Lf + Lr)))*(Lf + Lr)*((Lr**2*tan(d)**2)/(Lf + Lr)**2 + 1)**(1/2)) - 1/rho]])*ds

def jacobianC(u,rho,psi,ds):
	
	v=u[0]
	d=u[1]

	return np.array(
	[[Lr/(rho*(1 - Lr**2/rho**2)**(1/2)) + (Lr*arctan((Lf + Lr)/(rho*(1 - Lr**2/rho**2)**(1/2)))*(Lf**2 + 2*Lr*Lf + rho**2))/((Lr**2 - rho**2)*(Lf + Lr))],
	[1/(rho*(1 - Lr**2/rho**2)**(1/2)) - 1/rho + (arctan((Lf + Lr)/(rho*(1 - Lr**2/rho**2)**(1/2)))*(Lf**2 + 2*Lr*Lf + rho**2))/((Lr**2 - rho**2)*(Lf + Lr))]])*ds

def jacobianF(u,rho,psi, ds):

	v=u[0]
	d=u[1]

	return np.eye(stateLength)+np.array(
[[ -Lr/(rho**2*(1 - Lr**2/rho**2)**(1/2)), -rho**2/(Lr**2 - rho**2)], 
 [-1/(rho**2*(1 - Lr**2/rho**2)**(1/2))  ,    -Lr/(Lr**2 - rho**2)]])*ds

def jacobianH(u,rho,psi ,ds):

	v=u[0]
	d=u[1]

	return np.array([[ 0, -(Lr*(Lf**2 + 2*Lr*Lf + rho**2))/((Lr**2 - rho**2)*(Lf + Lr))],
			[ 0,      -(Lf**2 + 2*Lr*Lf + rho**2)/((Lr**2 - rho**2)*(Lf + Lr))]])*ds


def exit_gracefully(signum, frame):

	global running, maxtimeDuration, mintimeDuration

	running = False
	
	original_sigint = signal.getsignal(signal.SIGINT)
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)
	
	# restore the exit gracefully handler here	
	signal.signal(signal.SIGINT, exit_gracefully)

	print(maxtimeDuration, mintimeDuration)

def getAngles(position, orientation, velocity, angularVelocity):
	angles=transforms3d.euler.quat2euler(np.array([orientation.w,orientation.x,orientation.y,orientation.z]))
	rot=np.array(transforms3d.euler.euler2mat(angles[0],angles[1],angles[2]))

	psi=np.arctan2(velocity.y,velocity.x)#Slip

	alpha=np.arctan2(rot[0,1], rot[0,2])
	beta=np.arctan2(rot[0,0], rot[0,2])
	theta=np.arctan2(-rot[2,1], -rot[2,0])

	return psi, alpha, beta, theta

def control(x, y, psi, beta):

	global startTime, velocity, accum, ySigmaPrev, Kp, Ki, Kd, pubySigma, Jessica, CC, Schmidt, CC2, xSigmaPrev, timeDuration, controlInput, deltaSigma, N, stateLength, controlLength, Lf, Lr, T, K, Q, R, S, vMin, vMax, sMin, sMax, g, h, maxtimeDuration, mintimeDuration

	cc=CC
	[phi, xSigma, ySigma, psiSigma, dtSigma]=traj(x, y, velocity, psi, beta, cc)
	deltaSigma=xSigma-xSigmaPrev
	xSigmaPrev=xSigma
	pubySigma.publish(ySigma)
	
	x=np.array([[ySigma], [psiSigma], [controlInput[0]], [controlInput[1]]])
	r=cvxopt.sparse([cvxopt.matrix(np.array([[0.0], [0.0]])) for i in range(0,N)])

	#print(dtSigma*deltaTime)
	#print(deltaSigma)

	ds=1e-1

	B=jacobianH(cvxopt.matrix(controlInput), cc.rho(phi), psi , ds)
	C=np.eye(stateLength)
	A=jacobianF(cvxopt.matrix(controlInput), cc.rho(phi), psi, ds)

	fbar=jacobianC(cvxopt.matrix(controlInput), cc.rho(phi), psi , ds)
	Cbar=np.array([[fbar[0][0]], [fbar[1][0]], [0], [0]])

	try:		
		S = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
		K = np.matrix(np.linalg.inv(B.T*S*B+R)*(B.T*S*A))
		Control, predictedStates=getControl(A, B, C, x, r, g1, g2, h1, h2, stateLength, controlLength, N, Q, R, S, Cbar)
	except(ArithmeticError, ValueError, np.linalg.linalg.LinAlgError):
		print("phi:"+str(phi)+"    xSigma:"+str(xSigma)+"    ySigma:"+str(ySigma)+"    psiSigma:"+str(psiSigma))
		x=np.array([[ySigma], [psiSigma], [0]])
		return controlInput+np.array(controlInput[0:controlLength])
	x=np.array([[ySigma], [psiSigma], [0]])
	controlInput=controlInput+Control[0:controlLength]
	print("phi:"+str(phi)+"    xSigma:"+str(xSigma)+"    ySigma:"+str(ySigma)+"    psiSigma:"+str(psiSigma))
	print("Frequency:"+str(1/timeDuration))
	print("Time Duration:"+str(timeDuration))
	print("Elapsed Time:"+str(time.time()-startTime))

	if(timeDuration>maxtimeDuration):
		maxtimeDuration=timeDuration
	
	if(timeDuration<mintimeDuration):
		mintimeDuration=timeDuration

	return np.array(controlInput)

def traj(x, y, v, psi, beta, CC):
	
	CC.setCoordinates(x,y)
	phi, cost, jac=np.squeeze(CC.getCoordinates())+0.01

	xt=np.squeeze(CC.X(phi))
	yt=np.squeeze(CC.Y(phi))
	tangent=CC.tangent(phi)/np.linalg.norm(CC.tangent(phi))
	psit=np.squeeze(arctan2(tangent[1], tangent[0]))
	normal=np.squeeze(np.array([tangent[1],-tangent[0]]))

	xSigma=np.squeeze(scipy.integrate.quad(lambda x: np.sqrt(CC.tangent(x)[0]**2+CC.tangent(x)[1]**2), 0, phi)[0])
	ySigma=np.squeeze(cos(psit)*(y-yt) - sin(psit)*(x-xt))
	psiSigma=np.arctan2(np.sin(np.squeeze(psi-psit)), np.cos(np.squeeze(psi-psit)))

	dtSigma=np.squeeze(CC.rho(phi)*((v.x*cos(psi)+v.y*sin(psi))*cos(psiSigma)+ (v.x*-sin(psi)+v.y*cos(psi))*sin(psiSigma))/(CC.rho(phi)-ySigma))

	return [phi, xSigma, ySigma, psiSigma, dtSigma]		

def callbackOdom(msg):

	global pubThrottle, pubSteering, position, velocity, orientation, angularVelocity, timeDuration, ready

	timeDuration+=deltaTime
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
	
	global pubThrottle, pubSteering, position, velocity, orientation, angularVelocity, timeDuration

	if(ready):

		theta, alpha, beta, psi=getAngles(position, orientation, velocity, angularVelocity)

		controlInput=control(position.x,position.y,psi,psi-theta)
		controlInput[1]=np.arctan2(sin(controlInput[1]), cos(controlInput[1]))

		if(controlInput[1]>sMaxRobust):
			controlInput[1]=sMaxRobust
		elif(controlInput[1]<sMinRobust):
			controlInput[1]=sMinRobust

		print(controlInput)

		pubThrottle.publish(controlInput[0])
		pubSteering.publish(controlInput[1])
	else:
		return

def main():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, timeDuration, startTime, pubThrottle, pubSteering, pubySigma, Jessica, CC, Schmidt, CC2
	
	rospy.init_node('Data')
	startTime=time.time()

	rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)
	pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=1)
	pubSteering = rospy.Publisher('/ackermann/Steering', Float32, queue_size=1)

	pubySigma = rospy.Publisher('/ackermann/ySigma', Float32, queue_size=10)

	X=lambda t: 10*cos(t)-10
	Y=lambda t: 10*sin(t)
	tangent=lambda t: np.array([-10*sin(t), 10*cos(t)])

	rho=lambda t: 10

	Jessica=CurvilinearCoordinates(X,Y,tangent,rho)

	R=10
	a=0.2
	b=3

	X1=lambda t: R*(a - 1) - R*cos(t)*(a*cos(b*t) - 1)
	Y1=lambda t: -R*sin(t)*(a*cos(b*t) - 1)
	tangent1=lambda t: np.array([R*sin(t)*(a*cos(b*t) - 1) + R*a*b*sin(b*t)*cos(t),
 R*a*b*sin(b*t)*sin(t) - R*cos(t)*(a*cos(b*t) - 1)])
	rho1= lambda t: (abs(R)*abs(a**2*cos(b*t)**2 - 2*a*cos(b*t) + a**2*b**2 - a**2*b**2*cos(b*t)**2 + 1)**(3/2))/abs(2*a*cos(b*t) - a**2*cos(b*t)**2 - 2*a**2*b**2 + a*b**2*cos(b*t) + a**2*b**2*cos(b*t)**2 - 1)


	CC=CurvilinearCoordinates(X1,Y1,tangent1,rho1)

	X2=lambda t: 0.1*(.05*t**2+0.15*t**3)
	Y2=lambda t: t
	tangent2=lambda t: np.array([0.1*(0.05*2*t+0.15*3*t**2), 1])
	rho2= lambda t: 1/((400*(abs((9*t + 1)*(4*t**2 - 36*t**3*sign(t*(9*t + 2))**2 - 81*t**4*sign(t*(9*t + 2))**2 - 4*t**2*sign(t*(9*t + 2))**2 + 36*t**3 + 81*t**4 + 40000*sign(t*(9*t + 2))**2))**2 + 40000*abs(t*(9*t + 2))**2*abs(sign(t*(9*t + 2))*(9*t + 1))**2*abs(sign(t*(9*t + 2)))**4)**(1/2))/(abs(sign(t*(9*t + 2)))**2*(abs(t*(9*t + 2))**2 + 40000)**2))
			
	Schmidt=CurvilinearCoordinates(X2,Y2,tangent1,rho2)

	global desiredSteeringAngle, desiredSpeed, position, rotation, velocity, angularVelocity
	signal.signal(signal.SIGINT, exit_gracefully)

	while(running):
		start_time = time.time()
		sendControls()
		timeDuration = time.time()-start_time

	sys.exit(1)
if __name__=="__main__":
	main()
