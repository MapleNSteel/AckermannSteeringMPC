import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos,arctan2
import vrep
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
deltaTime=1/10.

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

		[xe, ye, thetae, vDesired, thetaDesired]=traj(time.time()-startTime, x, y, theta, beta)

		print(xe, ye, thetae)
	
		desiredSpeed=vDesired*cos(thetae)+0.8*xe
		td=thetaDesired+vDesired*(0.8*ye+0.5*sin(thetae))
		
		desiredSteeringAngle=arctan(1.2888*td/desiredSpeed)

		return [desiredSpeed,desiredSteeringAngle]

def traj(elapsedTime, x, y, theta, beta):
	
	t=elapsedTime
	w=2*pi*0.1*0.5
	R=5

	xt=R*(1-cos(w*t))
	yt=R*sin(w*t)

	xe=cos(theta)*(xt-x) + sin(theta)*(yt-y)
	ye=-sin(theta)*(xt-x) + cos(theta)*(yt-y)

	xd=R*w*sin(w*t)
	yd=R*w*cos(w*t)

	xdd=R*w*w*cos(w*t)
	ydd=-R*w*w*sin(w*t)

	thetae=arctan2(yd,xd)-theta

	thetaDesired=(-yd*xdd+xd*ydd)/(yd**2+xd**2)
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

	print("Theta"+str(theta))
	print("Alpha"+str(alpha))
	print("Beta"+str(beta))
	print("Gamma"+str(gamma))
	print()

def main():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime, startTime, pubThrottle, pubSteering
	
	rospy.init_node('Data')
	startTime=time.time()

	rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)
	pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=10)
	pubSteering = rospy.Publisher('/ackermann/Steering', Float32, queue_size=10)

	rate = rospy.Rate(1) # 1000hz

	global desiredSteeringAngle, desiredSpeed, position, rotation, velocity, angularVelocity
	signal.signal(signal.SIGINT, exit_gracefully)

	while(running):
		elapsedTime+=deltaTime
		rate.sleep()
if __name__=="__main__":
	main()
