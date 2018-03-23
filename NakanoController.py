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

from getVehicleStates import getAngles

running=True

class NakanoController(Controller):
	
	def callbackOdom(self, msg):

		global pubThrottle, pubSteeringAngle

		self.odom=msg

		position=Pose.position
		orientation=Pose.orientation

		velocity=Twist.linear
		angularVelocity=Twist.angular	

		theta, alpha, beta, gamma=getAngles(position, orientation, velocity, angularVelocity)

		self.controlInput=control(position.x,position.y,theta,beta)
		pubThrottle.pub(self.controlInput[0]) 
		pubSteeringAngle.pub(self.controlInput[1])

	def __init__():

		global pubThrottle, pubSteeringAngle
	
		rospy.Subscriber("/ackermann/Odom", Odometry, callbackOdom)	

		pubThrottle = rospy.Publisher('/ackermann/Throttle', Float32, queue_size=10)
		pubSteeringAngle = rospy.Publisher('/ackermann/SteeringAngle', Float32, queue_size=10)

	def control(self, x,y,theta,beta):

		desiredSpeed=0
		desiredSteeringAngle=0

		return [desiredSpeed,desiredSteeringAngle]
