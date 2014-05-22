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
import tf
#For rosbag:
import subprocess
import os
import signal
import copy


#There must be a planning scene or FK / IK crashes
def setupPlanningScene():
    print 'Waiting for set planning scene service...'
    rospy.wait_for_service('/environment_server/set_planning_scene_diff')
    setPlan = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
    req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
    setPlan(req)
    print 'OK'


class MotionKind:
    NOTHING = 0
    ARMS_TO_SIDE = 1
    SHAKE_GRIPPER = 2
    SHAKE_GRIPPER_FAST = 3


class TeleopCart:

    def __init__(self, joy_kind=JoyKind.default, base_path="data/bagfiles"):

        print "Joy kind: "+str(joy_kind)

        setupPlanningScene()

        self.mu = moveUtils.MoveUtils()

        self.joy_kind = joy_kind
        rospy.Subscriber("joy2", sensor_msgs.msg.Joy, self.joystickCallback);
        #self.joy1_pub = rospy.Publisher("joy", sensor_msgs.msg.Joy)
        #self.joy2_pub = rospy.Publisher("joy2", sensor_msgs.msg.Joy)

        self.init = True
        self.reset = True
        self.curr_x=[[0.0]*7, [0.0]*7]
        self.whicharm = 1

        self.linear_speed = [0.01,0.01,0.01]
        #self.linear_speed = [0.005,0.005,0.005]
        self.angular_speed = [0.05,0.05,0.05]
        self.time_step = 0.05
        self.gripper_max_effort = [12.0,15.0]  # gripper's max effort when closing (right,left). -1: do not limit, 50: close gently
        self.gripper_max_effort_string = [50.0,50.0]

        self.deadman = False
        self.dx_vec = [0.0, 0.0, 0.0]  # linear velocity
        self.w_vec = [0.0, 0.0, 0.0]  # angular velocity
        self.motion_request = MotionKind.NOTHING  # predefined motion

        #Recorder setup:
        self.base_path = base_path+'/'
        self.file_path = base_path+'/'
        self.seg_num = 1
        self.bag_process = None
        self.recording = False
        self.SIGINT = signal.SIGINT

        #List of topics to be stored into a bag file:
        self.topic_list = ["/ar_world_model",
            "/l_arm_controller/state",
            "/r_arm_controller/state",
            "/l_arm_controller_loose/state",
            "/r_arm_controller_loose/state",
            "/l_gripper_controller/state",
            "/r_gripper_controller/state",
            "/pressure/l_gripper_motor",
            "/pressure/l_gripper_motor_info",
            "/pressure/r_gripper_motor",
            "/pressure/r_gripper_motor_info" ]

    def __del__(self):
        self.stopRecord()

    def joystickCallback(self, msg):

        self.dx_vec = [0.0, 0.0, 0.0]  # linear velocity
        self.w_vec = [0.0, 0.0, 0.0]  # angular velocity
        self.motion_request = MotionKind.NOTHING  # predefined motion

        if self.joy_kind == JoyKind.PS3:

            if msg.buttons[8] == 0:
                self.deadman = False
            else:
                if not self.deadman:
                    self.init = True
                    self.reset = True
                self.deadman = True

            if not self.deadman:  return

            if msg.buttons[11] == 0:
                self.dx_vec[0] += msg.axes[1]*self.linear_speed[0]
                self.dx_vec[1] += msg.axes[0]*self.linear_speed[1]
                self.dx_vec[2] += msg.axes[3]*self.linear_speed[2]
            else:
                self.w_vec[0] += -msg.axes[0]*self.angular_speed[0]
                self.w_vec[1] += msg.axes[1]*self.angular_speed[1]
                self.w_vec[2] += msg.axes[2]*self.angular_speed[2]

            if msg.buttons[7] == 1:
                self.whicharm = 1
                print "Arm has switched to left"
            if msg.buttons[5] == 1:
                self.whicharm = 0
                print "Arm has switched to right"

            if msg.buttons[13] == 1:
                if msg.buttons[11] == 0:
                    self.mu.arm[self.whicharm].commandGripper(0.09,self.gripper_max_effort[self.whicharm],False);
                else:
                    self.mu.arm[self.whicharm].commandGripper(0.09,self.gripper_max_effort_string[self.whicharm],False);
            if msg.buttons[15] == 1:
                if msg.buttons[11] == 0:
                    self.mu.arm[self.whicharm].commandGripper(0.0,self.gripper_max_effort[self.whicharm],False);
                else:
                    self.mu.arm[self.whicharm].commandGripper(0.0,self.gripper_max_effort_string[self.whicharm],False);

            #Move arms to side
            if msg.buttons[12] == 1:
                self.motion_request = MotionKind.ARMS_TO_SIDE
            if msg.buttons[14] == 1:
                if msg.buttons[11] == 0:
                    self.motion_request = MotionKind.SHAKE_GRIPPER
                else:
                    self.motion_request = MotionKind.SHAKE_GRIPPER_FAST

            if msg.buttons[3] == 1:
                self.startRecord()
            if msg.buttons[0] == 1:
                self.stopRecord()

        elif self.joy_kind == JoyKind.RS:

            if msg.buttons[7] == 0:
                self.deadman = False
            else:
                if not self.deadman:
                    self.init = True
                    self.reset = True
                self.deadman = True

            if not self.deadman:  return

            if msg.buttons[5] == 0:
                self.dx_vec[0] += msg.axes[1]*self.linear_speed[0]
                self.dx_vec[1] += msg.axes[0]*self.linear_speed[1]
                self.dx_vec[2] += msg.axes[3]*self.linear_speed[2]
            else:
                self.w_vec[0] += -msg.axes[0]*self.angular_speed[0]
                self.w_vec[1] += msg.axes[1]*self.angular_speed[1]
                self.w_vec[2] += msg.axes[2]*self.angular_speed[2]

            if(msg.axes[4] >= 0.9):
                self.whicharm = 1
                print "Arm has switched to left"
            elif(msg.axes[4] <= -0.9):
                self.whicharm = 0
                print "Arm has switched to right"

            if msg.buttons[1] == 1:
                if msg.buttons[5] == 0:
                    self.mu.arm[self.whicharm].commandGripper(0.09,self.gripper_max_effort[self.whicharm],False);
                else:
                    self.mu.arm[self.whicharm].commandGripper(0.09,self.gripper_max_effort_string[self.whicharm],False);
            if msg.buttons[3] == 1:
                if msg.buttons[5] == 0:
                    self.mu.arm[self.whicharm].commandGripper(0.0,self.gripper_max_effort[self.whicharm],False);
                else:
                    self.mu.arm[self.whicharm].commandGripper(0.0,self.gripper_max_effort_string[self.whicharm],False);

            #Move arms to side
            if msg.buttons[0] == 1:
                self.motion_request = MotionKind.ARMS_TO_SIDE
            if msg.buttons[2] == 1:
                if msg.buttons[5] == 0:
                    self.motion_request = MotionKind.SHAKE_GRIPPER
                else:
                    self.motion_request = MotionKind.SHAKE_GRIPPER_FAST

            if msg.buttons[8] == 1:
                self.startRecord()
            if msg.buttons[9] == 1:
                self.stopRecord()

        else:

            print "Invalid joystick kind: "+str(self.joy_kind)
            sys.exit(0)

        #dx_vec= [0.0]*7
        #dx_vec[0]= 0.02
        if self.dx_vec != [0.0]*3 or self.w_vec != [0.0]*3:
            print self.dx_vec, ' ', self.w_vec
        #self.controlCart(dx_vec)


    def controlCart(self):

        if self.motion_request != MotionKind.NOTHING:
            if self.motion_request == MotionKind.ARMS_TO_SIDE:
                self.moveArmsToSide()
            elif self.motion_request == MotionKind.SHAKE_GRIPPER:
                self.shakeGripper(shake_Hz=1.0,shake_width=0.06)
            elif self.motion_request == MotionKind.SHAKE_GRIPPER_FAST:
                self.shakeGripper(shake_Hz=2.0,shake_width=0.06)

            self.motion_request = MotionKind.NOTHING
            self.init = True
            self.reset = True

        if not self.deadman: return

        if self.reset:
            self.mu.arm[0].cart_exec.resetIK()
            self.mu.arm[1].cart_exec.resetIK()
            self.reset = False

        if self.dx_vec == [0.0]*3 and self.w_vec == [0.0]*3: return

        i = self.whicharm

        # If the previous goal is active, ignore the command
        if self.mu.arm[i].getCartTrajState() == al.GoalStatus.ACTIVE:
            #print "DEBUG"
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

            #self.init = False

            if err > 0.1:
                #self.mu.arm[0].moveToJointAngle(q[0])
                #self.mu.arm[1].moveToJointAngle(q[1])
                self.mu.arm[0].cart_exec.resetIK()
                self.mu.arm[1].cart_exec.resetIK()
                #return

        dx_vec_e = self.dx_vec+[0.0]*4
        cart_pos = [(np.array(self.curr_x[i])+np.array(dx_vec_e)).tolist()]
        w_vec_norm = la.norm(self.w_vec)
        if w_vec_norm > 1.0e-4:
            rot = tf.transformations.quaternion_about_axis(w_vec_norm,self.w_vec)
        else:
            rot = [0.,0.,0.,1.]
        cart_pos[0][3:7] = tf.transformations.quaternion_multiply(rot,cart_pos[0][3:7])

        print "Target= "+str(cart_pos)
        #grip_traj = [0.0]
        grip_traj = [self.mu.arm[i].getGripPoseInfo()[0]]  # keep the current position
        dt = self.time_step
        splice_time = rospy.Time.now()
        blocking = False
        self.mu.arm[i].followCartTraj(cart_pos, grip_traj, dt, splice_time, blocking)
        #self.mu.arm[i].cart_exec.followCartTraj(cart_pos, grip_traj, dt, q[i], splice_time, blocking)
        print "Error code= "+str(self.mu.arm[i].cart_exec.last_error_code)
        if self.mu.arm[i].cart_exec.last_error_code != 1:
            self.init = True
            self.reset = True
            #self.mu.arm[i].cart_exec.traj_client.wait_for_result()
        #time.sleep(dt)

        #q=[[],[]]
        #p=[[],[]]
        #q[i] = self.mu.arm[i].getCurrentPosition()  # current right-arm joint angles
        #p[i] = self.mu.arm[i].cart_exec.makeFKRequest(q[i])
        #self.curr_x[i] = p[i]
        #print "Result= "+str(p[i])
        self.curr_x[i] = cart_pos[0]


    def moveArmsToSide(self):
        self.deadman = False
        self.init = True
        self.reset = True
        #time.sleep(0.1)
        self.mu.arm[0].traj_client.wait_for_result()
        self.mu.arm[0].cart_exec.traj_client.wait_for_result()
        self.mu.arm[1].traj_client.wait_for_result()
        self.mu.arm[1].cart_exec.traj_client.wait_for_result()
        target = [[],[]]
        target[1] = [2.115, -0.020, 1.640, -2.070, 1.640, -1.680, 1.398]
        target[0] = [-2.115, 0.020, -1.640, -2.070, -1.640, -1.680, 1.398]
        threshold = 0.01
        print "Moving to home position"
        print "Left arm..."
        self.mu.arm[1].commandGripper(0.08, -1)
        q = np.array(self.mu.arm[1].getCurrentPosition())
        diff = la.norm(np.array(target[1])-q)
        if diff > threshold:
            self.mu.arm[1].moveToJointAngle(target[1], 4.0)
        else:
            print "Left arm is already at the goal ({diff})".format(diff=diff)
        print "Right arm..."
        self.mu.arm[0].commandGripper(0.08, -1)
        q = np.array(self.mu.arm[0].getCurrentPosition())
        diff = la.norm(np.array(target[0])-q)
        if diff > threshold:
            self.mu.arm[0].moveToJointAngle(target[0], 4.0)
        else:
            print "Right arm is already at the goal ({diff})".format(diff=diff)
        print "Done"


    def shakeGripper(self,shake_Hz=2.0,shake_width=0.04):
        self.deadman = False
        self.init = True
        self.reset = True
        i = self.whicharm
        self.mu.arm[i].traj_client.wait_for_result()
        self.mu.arm[i].cart_exec.traj_client.wait_for_result()
        q = self.mu.arm[i].getCurrentPosition()
        p = self.mu.arm[i].cart_exec.makeFKRequest(q)
        g = self.mu.arm[i].getGripPoseInfo()[0]
        p2 = copy.deepcopy(p)
        p2[2] += shake_width
        cart_pos=[p2,p,p2,p]
        grip_traj=[g,g,g,g]
        print cart_pos
        dt = 2.0/shake_Hz/5.0
        splice_time = rospy.Time.now()
        blocking = True
        self.mu.arm[i].followCartTraj(cart_pos, grip_traj, dt, splice_time, blocking)


    def startRecord(self):
        if not self.recording:
            #In the first recording, we make a folder
            if self.seg_num == 1:
                #Make a new folder for the demo named by date/time
                t = time.localtime()
                self.file_path = self.base_path + str(t.tm_year) + "_" + str(t.tm_mon) + "_" + str(t.tm_mday) + "_" + str(t.tm_hour) + "_" + str(t.tm_min) + "_" + str(t.tm_sec) + "/"
                os.makedirs(self.file_path)
                print "################################"
                print "Created recording directory: ",self.file_path
                print "################################"


            self.recording = True
            topics = ' '.join(self.topic_list)
            filename = self.file_path + "/part" + str(self.seg_num) + ".bag "
            print "################################"
            print "Beginning to record into: ",filename
            print "################################"
            command = "rosbag record -O " + filename + topics
            self.bag_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.seg_num += 1

        else:
            self.stopRecord()
            self.startRecord()


    def stopRecord(self):
        if self.recording:
            print "################################"
            print "Stopping recording; pid: ",self.bag_process.pid
            print "################################"
            self.recording = False

            #Make sure we kill the child process that rosbag spawns in addition to the parent subprocess
            pid = self.bag_process.pid
            os.killpg(pid, self.SIGINT)


if __name__ == '__main__':
    rospy.init_node('teleopCartNode')
    joy_kind = rospy.get_param('~joy_kind', 'default')
    base_path = rospy.get_param('~base_path', 'data/bagfiles')
    tc = TeleopCart(strToJoyKind(joy_kind), base_path)
    r = rospy.Rate(150)
    #r = rospy.Rate(50)
    while not rospy.is_shutdown():
      tc.controlCart()
      r.sleep()
    #rospy.spin()

