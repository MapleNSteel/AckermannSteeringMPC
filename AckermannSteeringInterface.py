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

r=0.5*6.3407e-01
d=0.755
l=2.5772

desiredSteeringAngle=0
desiredSpeed=0/(r)

#State-Variables
odom=Odometry()

body_name='nakedAckermannSteeringCar'
joint_names = ['nakedCar_steeringLeft','nakedCar_steeringRight']
throttle_joint = ['nakedCar_motorLeft','nakedCar_motorRight']

Lr=1.2888
Lf=1.2884

deltaTime = 1./100
elapsedTime=0

def h(x):
	return x
def f(x,u):
	xDot=np.array(np.zeros((1,3)))

	beta=arctan((Lr/(Lf+Lr))*tan(u[1]))

	xDot[0,0]=u[0]*cos(x[2]+beta)
	xDot[0,1]=u[0]*sin(x[2]+beta)	
	xDot[0,2]=(u[0]/Lr)*sin(beta)

	return (x+ xDot*deltaTime)[0]

def jacobianF(x,u):

	beta=arctan((Lr/(Lf+Lr))*tan(u[1]))

	return np.array(np.eye(3))+np.array([[0,0,-u[0]*sin(x[2]+beta)*(u[0]/Lr)*sin(beta)],[0,0,u[0]*cos(x[2]+beta)*(u[0]/Lr)*sin(beta)],[0,0,0]])

def jacobianH(x):
	return np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])

F=jacobianF
H=jacobianH
P=np.zeros(3)

p_sigma=1e-5
o_sigma=1e-5	

Q=np.eye(3)*(p_sigma**2)
R=np.eye(3)*(o_sigma**2)

def exit_gracefully(signum, frame):

	running = False
	global clientID
	# stop the simulation
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

	# Before closing the connection to V-REP,
	# make sure that the last command sent out had time to arrive.
	vrep.simxGetPingTime(clientID)

	# Now close the connection to V-REP:
	vrep.simxFinish(clientID)
	print('connection closed...')

	
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
	
def openPort():
	global clientID
	# close any open connections
	vrep.simxFinish(-1)
	# Connect to the V-REP continuous server
	clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)
def startSim():
	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle

	vrep.simxSetFloatingParameter(clientID,
	        vrep.sim_floatparam_simulation_time_step,
	        deltaTime, # specify a simulation time step
	        vrep.simx_opmode_oneshot)

	# --------------------- Start the simulation

	# start our simulation in lockstep with our code
	vrep.simxSynchronous(clientID,True)
	vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)

	joint_handles = [vrep.simxGetObjectHandle(clientID,
	    name, vrep.simx_opmode_blocking)[1] for name in joint_names]

	throttle_handles = [vrep.simxGetObjectHandle(clientID,
	    name, vrep.simx_opmode_blocking)[1] for name in throttle_joint]
	
	body_handle = vrep.simxGetObjectHandle(clientID,
	    body_name, vrep.simx_opmode_blocking)

def setVehicleState(desiredSteeringAngle, desiredSpeed):
	
	global Pose
	
	if(desiredSteeringAngle!=0):
		steeringAngleLeft=np.arctan(l/(-d+l/np.tan(desiredSteeringAngle)))
		steeringAngleRight=np.arctan(l/(d+l/np.tan(desiredSteeringAngle)))
	else:
		steeringAngleLeft=0
		steeringAngleRight=0

	vrep.simxSetJointTargetPosition(clientID,
		joint_handles[0],
		steeringAngleLeft, # force to apply
		vrep.simx_opmode_streaming)
	vrep.simxSetJointTargetPosition(clientID,
		joint_handles[1],
		steeringAngleRight, # force to apply
		vrep.simx_opmode_streaming)

	vrep.simxSetJointTargetVelocity(clientID,
		joint_handles[0],
		desiredSpeed, # force to apply
		vrep.simx_opmode_streaming)
	vrep.simxSetJointTargetVelocity(clientID,
		joint_handles[1],
		desiredSpeed, # force to apply
		vrep.simx_opmode_streaming)

	vrep.simxSetJointTargetVelocity(clientID,
		throttle_handles[0],
		desiredSpeed, # force to apply
		vrep.simx_opmode_streaming)
	vrep.simxSetJointTargetVelocity(clientID,
		throttle_handles[1],
		desiredSpeed, # force to apply
		vrep.simx_opmode_streaming)

def getVehicleState():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime

	ret, xyz=vrep.simxGetObjectPosition(clientID, body_handle[1], -1, vrep.simx_opmode_streaming)
	Vxyz=vrep.simxGetObjectVelocity(clientID, body_handle[1], vrep.simx_opmode_streaming)
	ret, angles=vrep.simxGetObjectOrientation(clientID, body_handle[1], -1, vrep.simx_opmode_streaming)

	rot=np.array(transforms3d.euler.euler2mat(angles[0],angles[1],angles[2]))

	alpha=np.arctan2(-rot[2,1], -rot[2,0])
	beta=np.arctan2(rot[0,1], rot[0,2])

	Pose=np.array([xyz[0],xyz[1],alpha])
	EKF.predict(np.array([desiredSpeed, desiredSteeringAngle]))
	EKF.update(Pose)#update Kalman
	x,P=EKF.getPrediction()

	rot[2,0]=-np.cos(x[2])
	rot[2,1]=-np.sin(x[2])*cos(beta)
	rot[2,2]=np.sin(x[2])*sin(beta)

	#Substituting Estimations
	xyz[0]=x[0]
	xyz[1]=x[1]
	angles=np.array(transforms3d.euler.mat2euler(rot))

	position=np.array(xyz)
	orientation=np.array(transforms3d.euler.euler2quat(angles[0],angles[1],angles[2]))
	velocity=Vxyz[1]
	angularVelocity=Vxyz[2]

	odom.header.stamp.secs=int(elapsedTime)
	odom.header.stamp.nsecs=int((elapsedTime-int(elapsedTime))*1e9)
	
	odom.pose.pose.position.x=position[0]
	odom.pose.pose.position.y=position[1]
	odom.pose.pose.position.z=position[2]
	
	odom.pose.pose.orientation.w=orientation[0]
	odom.pose.pose.orientation.x=orientation[1]
	odom.pose.pose.orientation.y=orientation[2]
	odom.pose.pose.orientation.z=orientation[3]

	odom.twist.twist.linear.x=velocity[0]
	odom.twist.twist.linear.y=velocity[1]
	odom.twist.twist.linear.z=velocity[2]
	
	odom.twist.twist.angular.x=angularVelocity[0]
	odom.twist.twist.angular.y=angularVelocity[1]
	odom.twist.twist.angular.z=angularVelocity[2]

	pubOdom.publish(odom)

	#print("Position:        "+str(position))
	#print("Orientation:     "+str(orientation))
	#print("Velocity:        "+str(velocity))
	#print("Angular Velocity:"+str(angularVelocity))
	#print("")

def callbackThrottle(msg):
	global desiredSpeed
	desiredSpeed=float(msg.data)/r
def callbackSteering(msg):
	global desiredSteeringAngle
	desiredSteeringAngle=float(msg.data)

def initialisePose():
	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, Pose

	ret, xyz=vrep.simxGetObjectPosition(clientID, body_handle[1], -1, vrep.simx_opmode_blocking)
	ret, angles=vrep.simxGetObjectOrientation(clientID, body_handle[1], -1, vrep.simx_opmode_streaming)

	rot=np.array(transforms3d.euler.euler2mat(angles[0],angles[1],angles[2]))

	alpha=np.arctan2(-rot[2,1], -rot[2,0])

	Pose=np.array([xyz[0],xyz[1],alpha])

def main():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime
	
	rospy.init_node('Odom')

	rospy.Subscriber("/ackermann/Throttle", Float32, callbackThrottle)
	rospy.Subscriber("/ackermann/Steering", Float32, callbackSteering)	

	rate = rospy.Rate(100) # 1000hz
	pubOdom = rospy.Publisher('/ackermann/Odom', Odometry, queue_size=10)

	global desiredSteeringAngle, desiredSpeed, position, rotation, velocity, angularVelocity
	signal.signal(signal.SIGINT, exit_gracefully)

	try:
		global clientID, joint_handles, throttle_handles, d, l
		openPort()

		if clientID != -1: # if we connected successfully
			startSim()
			initialisePose()
			EKF=ExtendedKalmanFilter(jacobianF,Q,jacobianH,R,P,f,h,Pose)
			print ('Connected to remote API server')
	except:
		exit(0)
	while(running):
		setVehicleState(desiredSteeringAngle, desiredSpeed)
		getVehicleState()		

		elapsedTime+=deltaTime
		vrep.simxSynchronousTrigger(clientID)
		rate.sleep()
if __name__=="__main__":
	main()
