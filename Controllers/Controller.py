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

	def __init__():
	
		rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)	

		self.pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=10)
		self.pubSteeringAngle = rospy.Publisher('/ackermann/SteeringAngle', Float32, queue_size=10)

	def control(self, x,y,theta,beta):

		desiredSpeed=0
		desiredSteeringAngle=0

		return [desiredSpeed,desiredSteeringAngle]
