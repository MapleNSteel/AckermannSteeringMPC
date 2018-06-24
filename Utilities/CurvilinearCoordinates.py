import time
import matplotlib.pyplot as plt
import math
from numpy import sin, cos, pi
import numpy as np
from scipy.optimize import minimize

class CurvilinearCoordinates:
	
	def __init__(self, X, Y, tangent, rho):
		self.X=X
		self.Y=Y
		self.tangent=tangent
		self.rho=rho

		self.x=0
		self.y=0
		self.theta=0
		self.cost=0

	def J(self,t):
		
		J=((self.X(t)-self.x)**2)+((self.Y(t)-self.y)**2)
		return J
	
	def jacobian(self, t):
		
		return self.tangent(t)[0]*2*(self.X(t)-self.x)+self.tangent(t)[1]*2*(self.Y(t)-self.y)

	def setCoordinates(self,x,y):
		self.x=x
		self.y=y

	def getCoordinates(self):
		self.theta = minimize(self.J, self.theta, jac=self.jacobian, method="Newton-CG").x
		return self.theta, self.J(self.theta), self.jacobian(self.theta)

def main():
	X=lambda t: 10*(1+0.2*cos(8*t))*cos(t)
	Y=lambda t: 10*(1+0.2*cos(8*t))*sin(t)
	tangent=lambda t: np.array([-2*8*sin(8*t)*cos(t)-10*(1+0.2*cos(8*t))*sin(t), -2*8*sin(8*t)*sin(t)+10*(1+0.2*cos(8*t))*cos(t)])
	rho=lambda t: (abs((49*cos(7*t) + 81*cos(9*t) + 10*cos(t))/(abs(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))^2 + abs(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))^2)^(1/2) + ((2*abs(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))*sign(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))*(81*sin(9*t) - 49*sin(7*t) + 10*sin(t)) - 2*abs(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))*sign(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))*(49*cos(7*t) + 81*cos(9*t) + 10*cos(t)))*(7*sin(7*t) + 9*sin(9*t) + 10*sin(t)))/(2*(abs(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))^2 + abs(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))^2)^(3/2)))^2 + abs((81*sin(9*t) - 49*sin(7*t) + 10*sin(t))/(abs(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))^2 + abs(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))^2)^(1/2) - ((2*abs(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))*sign(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))*(81*sin(9*t) - 49*sin(7*t) + 10*sin(t)) - 2*abs(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))*sign(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))*(49*cos(7*t) + 81*cos(9*t) + 10*cos(t)))*(9*cos(9*t) - 7*cos(7*t) + 10*cos(t)))/(2*(abs(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))^2 + abs(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))^2)^(3/2)))^2)^(1/2)/(abs(9*cos(9*t) - 7*cos(7*t) + 10*cos(t))^2 + abs(7*sin(7*t) + 9*sin(9*t) + 10*sin(t))^2)^(1/2)

	CC=CurvilinearCoordinates(X,Y,tangent,rho)

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
