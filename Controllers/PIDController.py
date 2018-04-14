#Can't handle lower speeds
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos,arctan2,sign,fmod,sqrt
import scipy
from time import sleep
import signal
import time
import sys
import rospy
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter
from CurvilinearCoordinates import *

running=True
elapsedtSigmaime=0
deltaTime=10/1000.


Lr=1.2888
Lf=1.2884

L=3.3138

X=[]
Y=[]

accum=0
ySigmaPrev=0
xSigmaPrev=0

Kp, Ki, Kd=10.0, 0.01, 20.0

T=0

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

	theta=np.arctan2(velocity.y,velocity.x)#Slip

	alpha=np.arctan2(rot[0,1], rot[0,2])
	beta=np.arctan2(rot[0,0], rot[0,2])
	gamma=np.arctan2(-rot[2,1], -rot[2,0])

	return theta, alpha, beta, gamma

def control(x, y, theta, beta):

		global startTime, velocity, accum, ySigmaPrev, Kp, Ki, Kd, pubySigma, CC, CC1, CC2, xSigmaPrev, T, elapsedtSigmaime

		[phi, xSigma, ySigma, thetaSigma, dtSigma]=traj(x, y, velocity, theta, beta, CC)
		accum=accum+ySigma
	
		desiredSpeed=2
		desiredSteeringAngle=-Kp*ySigma+-Ki*accum+-Kd*(ySigma-ySigmaPrev)
		ySigmaPrev=ySigma
		
		if(not dtSigma==0):
			T=T+((xSigma-xSigmaPrev)/dtSigma)
		xSigmaPrev=xSigma

		#print(desiredSpeed,desiredSteeringAngle)
		print("phi:"+str(phi)+"    xSigma:"+str(xSigma)+"    ySigma:"+str(ySigma)+"    thetaSigma:"+str(thetaSigma)+"    dtSigma:"+str(dtSigma)+"    T:"+str(T))
		
		if(desiredSteeringAngle<-np.pi/3):
			desiredSteeringAngle=-np.pi/3
		elif(desiredSteeringAngle>np.pi/3):
			desiredSteeringAngle=np.pi/3


		pubySigma.publish(T)

		return [desiredSpeed,desiredSteeringAngle]

def traj(x, y, v, theta, beta, cc):
	
	CC.setCoordinates(x,y)
	phi=CC.getCoordinates()

	xt=CC.X(phi)
	yt=CC.Y(phi)
	tangent=CC.tangent(phi)
	thetat=arctan2(tangent[1], tangent[0])
	normal=np.array([tangent[1],-tangent[0]])

	xSigma=scipy.integrate.quad(lambda x: np.sqrt(CC.tangent(x)[0]**2+CC.tangent(x)[1]**2), 0, phi)[0]
	ySigma=cos(thetat)*(y-yt) - sin(thetat)*(x-xt)
	thetaSigma=theta-thetat
	
	print(sqrt(v.x**2+v.y**2)*cos(beta+thetaSigma))
	print(v.x*cos(thetaSigma) - v.y*sin(thetaSigma))

	dtSigma=sqrt(v.x**2+v.y**2)*cos(beta+thetaSigma)/(1-ySigma/CC.rho(phi))

	return [phi, xSigma, ySigma, thetaSigma, dtSigma]	

def callbackOdom(msg):

	global pubThrottle, pubSteering, position, velocity, orientation, angularVelocity

	Pose=msg.pose.pose
	Twist=msg.twist.twist

	position=Pose.position
	orientation=Pose.orientation

	velocity=Twist.linear
	angularVelocity=Twist.angular	

	theta, alpha, beta, gamma=getAngles(position, orientation, velocity, angularVelocity)

	controlInput=control(position.x,position.y,gamma,theta-gamma)
	
	pubThrottle.publish(controlInput[0])
	pubSteering.publish(controlInput[1])
	
	#print("Theta"+str(theta))
	#print("Alpha"+str(alpha))
	#print("Beta"+str(beta))
	#print("Gamma"+str(gamma))
	#print()

def main():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedtSigmaime, startTime, pubThrottle, pubSteering, pubySigma, CC, CC1, CC2
	
	rospy.init_node('Data')
	startTime=time.time()

	rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)
	pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=10)
	pubSteering = rospy.Publisher('/ackermann/Steering', Float32, queue_size=10)

	pubySigma = rospy.Publisher('/ackermann/ySigma', Float32, queue_size=10)

	rate = rospy.Rate(100) # 1000hz

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
		elapsedtSigmaime+=deltaTime
		rate.sleep()
if __name__=="__main__":
	main()
