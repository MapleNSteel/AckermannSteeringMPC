import time
import matplotlib.pyplot as plt
import math
from numpy import sin, cos, pi
from scipy.optimize import minimize

class CurvilinearCoordinates:
	
	def __init__(self, X, Y, tangent):
		self.X=X
		self.Y=Y

		self.tangent=tangent

		self.x=0
		self.y=0
		self.theta=0

	def J(self,t):
		
		J=((self.X(t)-self.x)**2)+((self.Y(t)-self.y)**2)
		return J

	def setCoordinates(self,x,y):
		self.x=x
		self.y=y

	def getCoordinates(self):
		self.theta = math.fmod(minimize(self.J, self.theta, method='nelder-mead', options={'xtol': 1e-4}).x, 2*pi)
		return self.theta

def main():
	X=lambda t: 10*(1+0.2*cos(8*t))*cos(t)
	Y=lambda t: 10*(1+0.2*cos(8*t))*sin(t)
	tangent=lambda t: np.array([-2*8*sin(8*t)*cos(t)-10*(1+0.2*cos(8*t))*sin(t), -2*8*sin(8*t)*sin(t)+10*(1+0.2*cos(8*t))*cos(t)])

	CC=CurvilinearCoordinates(X,Y,tangent)

	for i in range(0,1000):
		t=i/100
		CC.setCoordinates(X(t),Y(t))
		start_time = time.time()
		theta=CC.getCoordinates()
		elapsed_time = time.time() - start_time
		print(elapsed_time)
		#print([X(t),Y(t),theta,math.fmod(i/100,2*pi)])
		

if __name__=="__main__":
	main()
