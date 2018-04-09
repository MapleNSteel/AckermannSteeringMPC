#RETIRED!!!!

import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos,arctan2
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

running=True
elapsedTime=0
deltaTime=1/100.


Lr=1.2888
Lf=1.2884

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

		global startTime

		[xe, ye, thetae, vDesired, thetaDesired]=traj1(time.time()-startTime, x, y, theta, beta)
	
		desiredSpeed=vDesired#*cos(thetae)+5*xe
		td=thetaDesired#+vDesired*(5*ye+10*sin(thetae))
	
		desiredSteeringAngle=arctan((Lf+Lr)*td/desiredSpeed)
		
		if(desiredSteeringAngle<-np.pi/4):
			desiredSteeringAngle=-np.pi/4
		elif(desiredSteeringAngle>np.pi/4):
			desiredSteeringAngle=-np.pi/4

def traj2(elapsedTime, x, y, theta, beta):
	
	global X,Y

	param1=np.array([0,1.5,0,-0.02])
	param2=np.array([0,0,0.3,-0.02])

	if(elapsedTime>=5.0):
		elapsedTime=5.0
		

	basis=np.array([1,elapsedTime,elapsedTime**2,elapsedTime**3])
	basisD=np.array([0,1,2*elapsedTime,3*elapsedTime**2])
	basisDD=np.array([0,0,2,3*2*elapsedTime])

	x=np.dot(param1,basis)
	y=np.dot(param2,basis)

	X.append(x)
	Y.append(y)

	xd=np.dot(param1,basisD)
	yd=np.dot(param2,basisD)
	
	xdd=np.dot(param1,basisDD)
	ydd=np.dot(param2,basisDD)

	thetad=arctan2(yd,xd)

	xe=cos(theta)*(xd-x) + sin(theta)*(yd-y)
	ye=-sin(theta)*(xd-x) + cos(theta)*(yd-y)

	thetae=thetad-theta

	thetaDesired=((-yd*xdd+xd*ydd)/(xd**2+yd**2))
	vDesired=(yd*sin(thetad)+xd*cos(thetad))

	#	print(thetaDesired,vDesired)

	return [xe, ye, thetae, vDesired, thetaDesired]

def traj1(elapsedTime, x, y, theta, beta):
	
	w=0.1
	t=elapsedTime
	T=0.000001
	
	tau=2*pi*w*(t+T*np.exp(-t/T))
	taud=2*pi*w*(1-np.exp(-t/T))
	taudd=2*pi*w*(np.exp(-t/T)/T)

	r=6
	c=3
	a=0.2
	b=1

	R=r*(b-a*cos(c*tau))
	Rd=r*a*sin(c*tau)*c*taud
	Rdd=r*a*cos(c*tau)*((c*taud)**2)+r*a*sin(c*tau)*c*taud

	xt=R*cos(tau)-r*(b-a)
	yt=R*sin(tau)

	xd=-R*sin(tau)*taud+Rd*cos(tau)
	yd=R*cos(tau)*taud+Rd*sin(tau)

	xdd=-R*cos(tau)*(taud**2)-R*sin(tau)*taudd+Rdd*cos(tau)-Rd*sin(tau)*taud
	ydd=-R*sin(tau)*(taud**2)+R*cos(tau)*taudd+Rdd*sin(tau)+Rd*cos(tau)*taud

	xe=cos(theta)*(xt-x) + sin(theta)*(yt-y)
	ye=-sin(theta)*(xt-x) + cos(theta)*(yt-y)

	thetae=arctan2(yd,xd)-theta

	thetaDesired=(-yd*xdd+xd*ydd)/((xd)**2+(yd)**2)
	vDesired=yd*sin(arctan2(yd,xd))+xd*cos(arctan2(yd,xd))

	return [xe, ye, thetae, vDesired, thetaDesired]

def traj(elapsedTime, x, y, theta, beta):
	
	w=0.1
	t=elapsedTime
	T=1
	
	tau=2*pi*w*(t+T*np.exp(-t/T))
	taud=2*pi*w*(1-np.exp(-t/T))
	taudd=2*pi*w*(np.exp(-t/T)/T)

	xt=0.1*tau
	yt=5*sin(0.5*tau)

	xd=0.5*taud
	yd=5*0.5*cos(0.5*tau)*taud

	xdd=0.5*taudd
	ydd=-5*0.5*0.5*sin(tau)*(taud**2)+5*0.5*cos(tau)*taudd

	xe=cos(theta)*(xt-x) + sin(theta)*(yt-y)
	ye=-sin(theta)*(xt-x) + cos(theta)*(yt-y)

	thetae=arctan2(yd,xd)-theta

	thetaDesired=(-yd*xdd+xd*ydd)/((xd)**2+(yd)**2)
	vDesired=yd*sin(arctan2(yd,xd))+xd*cos(arctan2(yd,xd))

	return [xe, ye, thetae, vDesired, thetaDesired]


def callbackOdom(msg):

	global pubThrottle, pubSteering

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

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime, startTime, pubThrottle, pubSteering
	
	rospy.init_node('Data')
	startTime=time.time()

	rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)
	pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=10)
	pubSteering = rospy.Publisher('/ackermann/Steering', Float32, queue_size=10)

	rate = rospy.Rate(100) # 1000hz

	global desiredSteeringAngle, desiredSpeed, position, rotation, velocity, angularVelocity
	signal.signal(signal.SIGINT, exit_gracefully)

	while(running):
		elapsedTime+=deltaTime
		rate.sleep()
if __name__=="__main__":
	main()
