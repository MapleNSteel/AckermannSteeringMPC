import numpy as np
from numpy import linalg as linalg
from KF import KalmanFilter

class ExtendedKalmanFilter(KalmanFilter):

	def __init__(self):
		#Parameters
		self.F=None
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
	def __init__(self, F, Q, H, R, P, f, h, x):

		self.F=F
		self.Q=Q
		self.H=H
		self.R=R
		#Outputs
		self.P=P
		self.f=f
		self.h=h
		self.x=x
		self.innovation=np.zeros(np.shape(x))
		self.residual=np.zeros(np.shape(x))
		self.y=np.zeros(np.shape(x))
		self.S=self.R+(self.H(self.x).dot(self.P)).dot(self.H(self.x).transpose())
	def predict(self, u):
		self.x=self.f(self.x,u)
		self.P=(self.F(self.x,u).dot(self.P)).dot(self.F(self.x,u).transpose())+self.Q

	def getPrediction(self):
		
		return self.x, self.P

	def update(self, z):
		
		self.innovation=z-self.h(self.x)
		self.S=self.R+(self.H(self.x).dot(self.P)).dot(self.H(self.x).transpose())
		self.K=(self.P.dot(self.H(self.x).transpose())).dot(linalg.matrix_power(self.S, -1))
		self.P=(np.eye(np.shape(self.K)[0])-self.K.dot(self.H(self.x))).dot(self.P)
		self.x=self.x+self.K.dot(self.innovation)
