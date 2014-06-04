#!/usr/bin/python
import sys
import math
import copy
import numpy as np
import numpy.linalg as la
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


def GetWedge(w):
  wedge= np.zeros((3,3))
  wedge[0,0]=0.0;    wedge[0,1]=-w[2];  wedge[0,2]=w[1]
  wedge[1,0]=w[2];   wedge[1,1]=0.0;    wedge[1,2]=-w[0]
  wedge[2,0]=-w[1];  wedge[2,1]=w[0];   wedge[2,2]=0.0
  return wedge

def Rodrigues(w, epsilon=1.0e-6):
  th= la.norm(w)
  if th<epsilon:  return np.identity(3)
  w_wedge= GetWedge(w *(1.0/th))
  return np.identity(3) + w_wedge * math.sin(th) + np.dot(w_wedge,w_wedge) * (1.0-math.cos(th))

def InvRodrigues(R, epsilon=1.0e-6):
  alpha= (R[0,0]+R[1,1]+R[2,2] - 1.0) / 2.0

  if (alpha-1.0 < epsilon) and (alpha-1.0 > -epsilon):
    return np.array([0.0,0.0,0.0])
  else:
    w= np.zeros(3)
    th = math.acos(alpha)
    tmp= 0.5 * th / math.sin(th)
    w[0] = tmp * (R[2,1] - R[1,2])
    w[1] = tmp * (R[0,2] - R[2,0])
    w[2] = tmp * (R[1,0] - R[0,1])
    return w


#Remove radian jumping in trajctory
#velocities are ignored
def AngleTrajSmoother(traj_points):
  if len(traj_points)==0:  return
  q_prev= np.array(traj_points[0].positions)
  q_offset= np.array([0]*len(q_prev))
  for jp in traj_points:
    q= jp.positions
    q_diff= np.array(q) - q_prev
    for d in range(len(q_prev)):
      if q_diff[d]<-math.pi:  q_offset[d]+=1
      elif q_diff[d]>math.pi:  q_offset[d]-=1
    q_prev= copy.deepcopy(q)
    q[:]= q+q_offset*2.0*math.pi


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
