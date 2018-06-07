# AckermannControl
Control of an Ackermann Steering Vehicle using Linear Model Predictive Control.

## Requirements:
1. scipy
2. cvxopt
3. ROS: rospy
4. transforms3d

## What this repo can provide:
1. A simplified Ackermann steering model simulation in V-Rep
2. A general framework for a curvilinear-coordinate system, and Matlab scripts for generating required functions
3. A general framework for linear model predictive control:
  a) A specific Linear Model Predictive Controller for trajectory tracking based on spatial reformulation for the simulated ackermann steering vehicle
  b) A specific PID Controller for trajectory tracking
