#! /usr/bin/env python

import time
import numpy as np
import numpy.linalg as la
import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy
import actionlib as al
import moveUtils
from sensor_msgs.msg import *
from joyKind import *
import kinematics_msgs.srv
import pr2_controllers_msgs.msg
import arm_navigation_msgs.srv


#There must be a planning scene or FK / IK crashes
def setupPlanningScene():
    print 'Waiting for set planning scene service...'
    rospy.wait_for_service('/environment_server/set_planning_scene_diff')
    setPlan = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
    req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
    setPlan(req)
    print 'OK'


class TeleopCart:

    def __init__(self, joy_kind=JoyKind.default):

        print "Joy kind: "+str(joy_kind)

        setupPlanningScene()

        self.mu = moveUtils.MoveUtils()

        self.joy_kind = joy_kind
        rospy.Subscriber("joy2", sensor_msgs.msg.Joy, self.joystickCallback);
        #self.joy1_pub = rospy.Publisher("joy", sensor_msgs.msg.Joy)
        #self.joy2_pub = rospy.Publisher("joy2", sensor_msgs.msg.Joy)

        self.init = True
        self.curr_x=[[0.0]*7, [0.0]*7]
        self.whicharm = 0

        self.linear_speed = [0.01,0.01,0.01]
        self.time_step = 0.05

        self.deadman = False
        self.dx_vec = [0.0, 0.0, 0.0]


    def joystickCallback(self, msg):

        self.dx_vec = [0.0, 0.0, 0.0]

        if self.joy_kind == JoyKind.PS3:

            rospy.logwarn("FIXME: implement TeleopCart:: for JoyKind.PS3!!")
            #dx_vec[0] += msg.axes[0]*0.1
            #dx_vec[1] += msg.axes[0]*0.1
            #dx_vec[2] += msg.axes[0]*0.1

        elif self.joy_kind == JoyKind.RS:

            if msg.buttons[7] == 0:  self.deadman = False
            else:                    self.deadman = True

            if not self.deadman:  return

            self.dx_vec[0] += msg.axes[1]*self.linear_speed[0]
            self.dx_vec[1] += msg.axes[0]*self.linear_speed[1]
            self.dx_vec[2] += msg.axes[3]*self.linear_speed[2]

            if(msg.axes[4] >= 0.9):
                self.whicharm = 1
                print "Arm has switched to left"
            elif(msg.axes[4] <= -0.9):
                self.whicharm = 0
                print "Arm has switched to right"

            if(msg.axes[5] >= 0.9):
                self.mu.arm[self.whicharm].makeGripperRequest(0.08,True);
            elif(msg.axes[5] <= -0.9):
                self.mu.arm[self.whicharm].makeGripperRequest(0.0,True);

            if msg.buttons[0] == 1:
                self.deadman = False
                self.init = True
                #time.sleep(0.1)
                self.mu.arm[0].traj_client.wait_for_result()
                self.mu.arm[0].cart_exec.traj_client.wait_for_result()
                self.mu.arm[1].traj_client.wait_for_result()
                self.mu.arm[1].cart_exec.traj_client.wait_for_result()
                print "Moving to home position"
                print "Left arm..."
                self.mu.arm[1].commandGripper(0.08, -1)
                self.mu.arm[1].moveToJointAngle([2.115, -0.020, 1.640, -2.070, 1.640, -1.680, 1.398], 4.0)
                print "Right arm..."
                self.mu.arm[0].commandGripper(0.08, -1)
                self.mu.arm[0].moveToJointAngle([-2.115, 0.020, -1.640, -2.070, -1.640, -1.680, 1.398], 4.0)
                print "Done"

        else:

            print "Invalid joystick kind: "+str(self.joy_kind)
            sys.exit(0)

        #dx_vec= [0.0]*7
        #dx_vec[0]= 0.02
        print self.dx_vec
        #self.controlCart(dx_vec)


    def controlCart(self):

        if not self.deadman: return

        i = self.whicharm

        # If the previous goal is active, ignore the command
        if self.mu.arm[i].getCartTrajState() == al.GoalStatus.ACTIVE:
            return

        if self.init:
            q=[[],[]]
            p=[[],[]]
            q[0] = self.mu.arm[0].getCurrentPosition()  # current right-arm joint angles
            q[1] = self.mu.arm[1].getCurrentPosition()  # current left-arm joint angles
            p[0] = self.mu.arm[0].cart_exec.makeFKRequest(q[0])
            p[1] = self.mu.arm[1].cart_exec.makeFKRequest(q[1])
            #print "q= "+str(q)
            #print "p= "+str(p)
            err = la.norm(np.array(self.curr_x[i])-np.array(p[i]))
            print "DIFF= "+str(err)
            self.curr_x[0] = p[0]
            self.curr_x[1] = p[1]

            if err > 0.1:
                #self.mu.arm[0].moveToJointAngle(q[0])
                #self.mu.arm[1].moveToJointAngle(q[1])
                self.mu.arm[0].cart_exec.resetIK()
                self.mu.arm[1].cart_exec.resetIK()
                #return

        dx_vec_e = self.dx_vec+[0.0]*4
        cart_pos = [(np.array(self.curr_x[i])+np.array(dx_vec_e)).tolist()]
        self.curr_x[i] = cart_pos[0]
        print "Target= "+str(cart_pos)
        grip_traj = [0.0]
        dt = self.time_step
        splice_time = rospy.Time.now()
        blocking = False
        self.mu.arm[i].followCartTraj(cart_pos, grip_traj, dt, splice_time, blocking)
        #self.mu.arm[i].cart_exec.followCartTraj(cart_pos, grip_traj, dt, q[i], splice_time, blocking)
        print "Error code= "+str(self.mu.arm[i].cart_exec.last_error_code)
        if self.mu.arm[i].cart_exec.last_error_code != 1:
            self.init = True
        #time.sleep(dt)

        q[i] = self.mu.arm[i].getCurrentPosition()  # current right-arm joint angles
        p[i] = self.mu.arm[i].cart_exec.makeFKRequest(q[i])
        self.curr_x[i] = p[i]
        print "Result= "+str(p[i])


if __name__ == '__main__':
    rospy.init_node('teleopCartNode')
    tc = TeleopCart(strToJoyKind(rospy.get_param('~joy_kind', 'default')))
    r = rospy.Rate(200)
    while not rospy.is_shutdown():
      tc.controlCart()
      r.sleep()
    rospy.spin()


