import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos
from time import sleep
import signal
import time
import sys
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import transforms3d

running=True

class Controller:
	
	def callbackOdom(self, msg):

		global pubThrottle, pubSteeringAngle

		self.odom=msg

		x=odom.pose.pose.position.x
		y=odom.pose.pose.position.y

		theta=orientation=np.array(transforms3d.euler.quat2euler([odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z]))[1]
		self.controlInput=control(x,y,theta)
		pubThrottle.pub(self.controlInput[0]) 
		pubSteeringAngle.pub(self.controlInput[1])

	def __init__():

		global pubThrottle, pubSteeringAngle
	
		rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)	

		pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=10)
		pubSteeringAngle = rospy.Publisher('/ackermann/SteeringAngle', Float32, queue_size=10)

	def control(self, x,y,theta):

		desiredSpeed=0
		desiredSteeringAngle=0

		return [desiredSpeed,desiredSteeringAngle]
