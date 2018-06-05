# AckermannControl
Control of an Ackermann Steering Vehicle using Robust Linear Model Predictive Control.

## Requirements:
1. scipy
2. cvxopt
3. ROS: rospy
4. transforms3d

## What this repo can provide:
1. A simplified Ackermann steering model simulation in V-Rep
2. A general framework for a curvilinear-coordinate system, and Matlab scripts for generating required functions
3. A general framework for linear model predictive control:&nbsp;
  a) A Linear Model Predictive Controller for trajectory tracking based on spatial reformulation for the simulated ackermann steering vehicle, with input and state constraints&nbsp;
  b) An mRPI (minimal Robust Poisitvely Invariant) set computation code in matlab, for determining robust constraints       whilst accounting for noise&nbsp;
  b) A PID Controller for trajectory tracking&nbsp;
