#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Scott Niekum
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Scott Niekum

import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy 
import numpy as np
import matplotlib.pyplot as plt
import pickle
#import coordFrame 
import generalUtils
import trajUtils
import moveUtils
import drawUtils
import arWorldModel
import skillParse
import sys
import os
import os.path
import dmpExec
from cmnUtils import *



#Calculate the distance from the goal to determine convergence
def isConverged(curr_pos, goal_pos, goal_thresh):
    conv = True
    for i in range(0,7):
        diff = math.fabs(curr_pos[i] - goal_pos[i])
        if diff > goal_thresh[i]: 
            conv = False
    return conv

      
if __name__ == '__main__':

    RIGHT_ARM = 0
    LEFT_ARM = 1

    usage='''Replay a recorded trajectory
    USAGE: {exe} [OPTION] DIR_NAME SKILL_ID
      DIR_NAME:     directory where a demo file is located
      SKILL_ID:     index of the demo file ({skill_id})
      OPTION:
        -r          right arm (OFF)
        -l          left arm (ON)
        -cf XXX     control frame; -1 : wrist_roll_joint, otherwise use
                    ID of marker ({control_frame})
        -gf XXX     goal frame; -1 : torso_lift_link, otherwise use ID
                    of marker ({goal_frame})
        -plan       plan only (OFF)
        -goal X Y Z specify a new goal ({new_goal})
        -help       show help
    EXAMPLE:
      {exe} data/bagfiles/test2/a
      {exe} -r data/bagfiles/test2/a 1 -goal 0.57 -0.206 -0.0135'''

    basename = ''
    skill_id = 1
    whicharm = LEFT_ARM
    control_frame = -1
    goal_frame = -1
    plan_only = 0
    new_goal = []

    usage = usage.format(exe=sys.argv[0], skill_id=skill_id, control_frame=control_frame, goal_frame=goal_frame, new_goal=new_goal)
    #Parse option
    notag_opt=0
    it= iter(sys.argv)
    it.next() # skip exec name
    while True:
        try:
            a= it.next()
            if a=='-help' or a=='--help': print usage; sys.exit(0)
            elif a=='-r': whicharm = RIGHT_ARM
            elif a=='-l': whicharm = LEFT_ARM
            elif a=='-cf': control_frame = int(it.next())
            elif a=='-gf': goal_frame = int(it.next())
            elif a=='-plan': plan_only = 1
            elif a=='-goal': new_goal = [float(it.next()), float(it.next()), float(it.next())]
            else:
                if notag_opt==0: basename = a+'/'
                elif notag_opt==1: skill_id = int(a)
                else: print usage; sys.exit(0)
                notag_opt+=1
        except StopIteration:
            break
    print '-------------------------'
    print 'basename = ',basename
    print 'skill_id = ',skill_id
    print 'whicharm = ',whicharm
    print 'control_frame = ',control_frame
    print 'goal_frame = ',goal_frame
    print 'plan_only = ',plan_only
    print 'new_goal = ',new_goal
    print '-------------------------'

    if basename=='': print usage; sys.exit(0)

    print 'Do you continue?'
    if not ask_yes_no():  sys.exit(0)

    ##Parameters that select a skill to execute and the marker frame it is in (-1 for torso)
    #if (len(sys.argv) >= 6):
        #whicharm = int(sys.argv[1])
        #skill_id = int(sys.argv[2])
        #control_frame = int(sys.argv[3])
        #goal_frame = int(sys.argv[4])
        #plan_only = int(sys.argv[5])
        #if len(sys.argv) >= 7: basename = sys.argv[6]+'/'
        #if len(sys.argv) >= 10:
            #new_goal = [0]*3
            #new_goal[0] = float(sys.argv[7])
            #new_goal[1] = float(sys.argv[8])
            #new_goal[2] = float(sys.argv[9])
    #else:
        #print "\nAborting! Wrong number of command line args"
        #print "Usage: python singleReplay <whicharm> <skill_id> <control_frame> <goal_frame> <plan_only> (<base_path>)"
        #print "0 for right arm, 1 for left arm"
        #print "-1 for control frame: wrist_roll_joint ; -1 for goal_frame: torso_lift_link ; otherwise use ID of marker\n"
        #sys.exit(0)


    try:
    
        rospy.init_node('SingleReplayNode')
        #rospy.on_shutdown(shutdown_cb)
   
        #Setup utilities and world model
        gen_utils = generalUtils.GeneralUtils()
        traj_utils = trajUtils.TrajUtils()
        move_utils = moveUtils.MoveUtils()
        draw_utils = drawUtils.DrawUtils()
        wm = arWorldModel.ARWorldModel()

        #Construct file names
        demofile = basename + 'demo' + str(skill_id) + '.bag'
        picklefile = basename + 'Pickle' + str(skill_id) + '.txt'
        matfile = basename + 'Mat' + str(skill_id) + '.txt'
        markerfile = basename + 'Marker' + str(skill_id) + '.txt'


        #################### Load data and process trajectories ####################
        
        #Get the traj of the gripper
        arm_control_data = skillParse.singleSkillParse(picklefile)

        #Get the traj of the goal marker, if approprite
        if(goal_frame >= 0):
            marker_goal_data = traj_utils.createMarkerTrajFromFile(markerfile, goal_frame, -1)
        
        #If a control frame other than the gripper is specified, get that marker's trajectory and add on gripper info
        if (control_frame >= 0):
            #Estimate the rigid transform between marker and wrist and infer the rest from arm pose
            traj_data = traj_utils.inferMarkerTrajFromRigidTrans(arm_control_data, markerfile, control_frame)
        #Otherwise, get the whole skill demo from the arm traj
        else:
            traj_data = arm_control_data

        demo_start = traj_data[0]
        demo_goal = traj_data[-1]
        print "Demo goal: ", demo_goal
        if len(new_goal) > 0:
            demo_goal[0:len(new_goal)] = new_goal
        print "Current goal: ", demo_goal


        #################### Learn DMP and prepare for replay ####################
        
        dmp_exec = dmpExec.DMPExec()
        (tau, dmp_list) = dmp_exec.learnDMP(traj_data)
        gf = goal_frame
        #no_exec = True
        
        if(goal_frame >= 0):
            dmp_exec.executeDMP(whicharm, tau, dmp_list, demo_start, demo_goal, plan_only_no_exec=plan_only, goal_frame=gf, marker_goal=marker_goal_data[0])
        else:
            dmp_exec.executeDMP(whicharm, tau, dmp_list, demo_start, demo_goal, plan_only_no_exec=plan_only, goal_frame=gf)     

     
      
           

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
            
            
