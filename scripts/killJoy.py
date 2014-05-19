#! /usr/bin/env python
import roslib; roslib.load_manifest('rospy')
import rospy
import subprocess
import os

if __name__ == '__main__':
    rospy.init_node('killJoyNode')
    node_name = rospy.get_param('~node_name', 'joy')
    command = "rosnode kill " + node_name
    subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

