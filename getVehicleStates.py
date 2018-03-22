import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos
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

def callbackOdom(msg):

	global pubAlpha, pubBeta, pubGamma, pubTheta

	Pose=msg.pose.pose
	Twist=msg.twist.twist

	position=Pose.position
	orientation=Pose.orientation

	velocity=Twist.linear
	angularVelocity=Twist.angular	

	angles=transforms3d.euler.quat2euler(np.array([orientation.w,orientation.x,orientation.y,orientation.z]))
	theta=np.arctan2(velocity.x,velocity.y)

	alpha=angles[0]
	beta= angles[1]+alpha
	gamma=angles[2]

	pubTheta.publish(theta)
	pubAlpha.publish(alpha)
	pubBeta.publish(beta)
	pubGamma.publish(gamma)

def main():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime, pubAlpha, pubBeta, pubGamma, pubTheta
	
	rospy.init_node('Data')

	rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)
	pubTheta=rospy.Publisher("/ackermann/Theta", Float32, queue_size=10)
	pubAlpha=rospy.Publisher("/ackermann/Alpha", Float32, queue_size=10)
	pubBeta=rospy.Publisher("/ackermann/Beta", Float32, queue_size=10)	
	pubGamma=rospy.Publisher("/ackermann/Gamma", Float32, queue_size=10)

	rate = rospy.Rate(1) # 1000hz
	pubOdom = rospy.Publisher('/ackermann/Odom', Odometry, queue_size=10)

	global desiredSteeringAngle, desiredSpeed, position, rotation, velocity, angularVelocity
	signal.signal(signal.SIGINT, exit_gracefully)

	while(running):
		elapsedTime+=deltaTime
		rate.sleep()
if __name__=="__main__":
	main()
