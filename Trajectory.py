import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos
from time import sleep
import signal
import time
import sys

class Trajectory:

	def __init__(self, func):
	
		self.func=func

	def getTarget(self, elapsedTime, x, y, theta, beta):
		
		return self.func(elapsedTime, x, y, theta, beta)
