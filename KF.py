import numpy as np
from numpy import linalg as linalg

class KalmanFilter:

	def __init__(self):
		#Parameters
		self.F=None
		self.B=None
		self.Q=None
		self.H=None
		self.R=None
		#Outputs
		self.P=None
		self.x=None
		self.innovation=None
		self.residual=None
		self.y=None
		self.S=None
	def __init__(self, F, B, Q, H, R, P, x):

		self.F=F
		self.B=B
		self.Q=Q
		self.H=H
		self.R=R
		#Outputs
		self.P=P
		self.x=x
		self.innovation=np.zeros(np.shape(x))
		self.residual=np.zeros(np.shape(x))
		self.y=np.zeros(np.shape(x))
		self.S=self.R+(self.H.dot(self.P)).dot(self.H.transpose())
	def predict(self, u):
		self.x=self.F.dot(self.x)+self.B.dot(u)
		self.P=(self.F.dot(self.P)).dot(self.F.transpose())+self.Q

	def getPrediction(self):
		
		return self.x, self.P

	def update(self, z):
		
		self.innovation=z-self.H.dot(self.x)
		self.S=self.R+(self.H.dot(self.P)).dot(self.H.transpose())
		self.K=(self.P.dot(self.H.transpose())).dot(linalg.matrix_power(self.S, -1))
		self.x=self.x+self.K.dot(self.innovation)
		self.P=(np.eye(np.shape(self.K)[0])-self.K.dot(self.H)).dot(self.P).dot((np.eye(np.shape(self.K)[0])-self.K.dot(self.H)).transpose())+self.K.dot(self.R).dot(self.K.transpose())
		self.residual=z-self.H.dot(self.x)
