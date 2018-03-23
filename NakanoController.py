import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos,arctan2
from time import sleep
import signal
import time
import sys
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import transforms3d
from Trajectory import Trajectory

from getVehicleStates import getAngles

running=True

class NakanoController:
	
	def callbackOdom(self, msg):

		global pubThrottle, pubSteeringAngle

		Pose=msg.pose.pose
		Twist=msg.twist.twist

		position=Pose.position
		orientation=Pose.orientation

		velocity=Twist.linear
		angularVelocity=Twist.angular	

		theta, alpha, beta, gamma=getAngles(position, orientation, velocity, angularVelocity)
	
		print("Gamma"+str(gamma))
		print()

		self.controlInput=self.control(position.x,position.y,gamma,theta-gamma)
	
		pubThrottle.publish(self.controlInput[0])
		pubSteeringAngle.publish(self.controlInput[1])

	def __init__(self, startTime, trajectory):

		global pubThrottle, pubSteeringAngle
		
		self.startTime=startTime
		self.trajectory=trajectory
	
		rospy.Subscriber("/ackermann/Odom", Odometry, self.callbackOdom)	

		pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=10)
		pubSteeringAngle = rospy.Publisher('/ackermann/Steering', Float32, queue_size=10)

	def control(self, x, y, theta, beta):

		[xe, ye, thetae, vDesired, thetaDesired]=self.trajectory.getTarget(time.time()-self.startTime, x, y, theta, beta)

		desiredSpeed=vDesired*cos(thetae)+0.1*xe
		td=thetaDesired+vDesired*(0.1*ye+0.1*sin(thetae))
		desiredSteeringAngle=arctan2(1.2888*td,desiredSpeed)

		return [desiredSpeed,desiredSteeringAngle]

def traj(elapsedTime, x, y, theta, beta):
	
	t=elapsedTime
	w=2*pi*0.1
	R=1

	xt=(R*(1-cos(w*t)))
	yt=(-R*sin(w*t))

	xe=cos(theta)*(xt-x) + sin(theta)*(yt-y)
	ye=-sin(theta)*(xt-x) + cos(theta)*(yt-y)

	xd=R*w*sin(w*t)
	yd=-R*w*cos(w*t)

	xdd=R*w*w*cos(w*t)
	ydd=R*w*w*sin(w*t)

	thetae=arctan2(yd,xd)-(theta+beta)

	thetaDesired=(-yd*xdd+xd*ydd)/(yd**2+xd**2)
	vDesired=yd*cos(w*t)+xd*sin(w*t)

	return [xe, ye, thetae, vDesired, thetaDesired]

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

def main():
		
	rospy.init_node('Controller')
	now=time.time()

	trajectory=Trajectory(traj)
	nakanoController=NakanoController(now, trajectory)

	signal.signal(signal.SIGINT, exit_gracefully)

	while(running):
		pass
if __name__=="__main__":
	main()
