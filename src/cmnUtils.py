#!/usr/bin/python
import sys
import math
import copy
import numpy as np
import roslib; roslib.load_manifest('rospy')
import rospy
import trajectory_msgs.msg


def ask_yes_no():
  while 1:
    sys.stdout.write('  (y|n) > ')
    ans= sys.stdin.readline().strip()
    if ans=='y' or ans=='Y':  return True
    elif ans=='n' or ans=='N':  return False


# Matlab-like mod function that returns always positive
def Mod(x, y):
  if y==0:  return x
  return x-y*math.floor(x/y)

#Convert radian to [-pi,pi)
def AngleMod1(q):
  return Mod(q+math.pi,math.pi*2.0)-math.pi

#Convert radian to [0,2*pi)
def AngleMod2(q):
  return Mod(q,math.pi*2.0)


#Interpolate (p1,v1)-(p2,v2) of duration with num_p points and store into traj_points
#(p1,v1) is not contained.  traj_points should have the size num_p
#FIXME: do not use velocities!!!
#def InterpolateLinearly1(traj_points,p1,v1,p2,v2,duration,num_p):
  #if len(traj_points)!=num_p:
    #rospy.logerr('Error in InterpolateLinearly1: len(traj_points)!=num_p')
    #sys.exit(1)
  #p= np.array(p1).copy()
  #v= np.array(v1).copy()
  #dt= duration/float(num_p)
  #dp= (p2-p1)/float(num_p)
  #dv= (v2-v1)/float(num_p)
  #for i in range(num_p):
    #p+= dp
    #v+= dv
    #traj_points[i].time_from_start= rospy.Duration(dt*(i+1.0))
    #traj_points[i].positions= p.copy()
    ##traj_points[i].velocities= v.copy()


#Interpolate (p1)-(p2) of duration with time_step and store into traj_points
#(p1) is not contained.  Velocities are automatically computed
#If rot_adjust is true, each dimension is considered as a revolute joint, and a shorter direction is automatically chosen
#If vel_limits is specified, the velocity is adjusted so that it does not exceed vel_limits.
#When the velocity is modified, the duration is also modified, which is returned
def InterpolateLinearly2(traj_points,p1,p2,duration,time_step,rot_adjust=False, vel_limits=[]):
  traj_points[:]= []
  v= (p2-p1)/duration
  if rot_adjust:
    for d in range(len(p1)):
      delta= p2[d]-p1[d]
      if math.fabs(delta)>math.pi:
        v[d]= AngleMod1(delta)/duration
  if len(vel_limits)>0:
    #Borrowed Scott's velocity modification
    ratios= [math.fabs(v[d]) / vel_limits[d] for d in range(len(v))]
    max_r= max(ratios)
    if max_r>1.0:
      v= v/max_r
      duration= duration*max_r
  t= 0.0
  jp= trajectory_msgs.msg.JointTrajectoryPoint()
  t+= time_step
  while t<duration:
    p= p1+v*t
    jp.time_from_start= rospy.Duration(t)
    jp.positions= p.copy()
    #jp.velocities= v.copy()
    traj_points.append(copy.deepcopy(jp))
    t+= time_step
  if t-time_step<=duration:
    p= p1+v*duration
    jp.time_from_start= rospy.Duration(duration)
    jp.positions= p.copy()
    #jp.velocities= v.copy()
    traj_points.append(copy.deepcopy(jp))
  return duration
