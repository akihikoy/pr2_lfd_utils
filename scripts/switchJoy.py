#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy
from sensor_msgs.msg import *
from joyKind import *


class SwitchJoy:

    def __init__(self, joy_kind=JoyKind.default):

        print "Joy kind: "+str(joy_kind)

        self.joy_kind = joy_kind
        rospy.Subscriber("joy_raw", sensor_msgs.msg.Joy, self.joystickCallback);
        self.joy1_pub = rospy.Publisher("joy", sensor_msgs.msg.Joy)
        self.joy2_pub = rospy.Publisher("joy2", sensor_msgs.msg.Joy)
        self.last_controller = 0

    def joystickCallback(self, msg):

        controller = 0

        if self.joy_kind == JoyKind.PS3:

            if(msg.buttons[8] == 0):  controller = 1
            else:                     controller = 2

        elif self.joy_kind == JoyKind.RS:

            if(msg.buttons[7] == 0):  controller = 1
            else:                     controller = 2

        else:

            print "Invalid joystick kind: "+str(self.joy_kind)
            return

        if controller==1:
            if self.last_controller == 2:
                  self.joy2_pub.publish(msg)
            self.joy1_pub.publish(msg)
        elif controller==2:
            if self.last_controller == 1:
                  self.joy1_pub.publish(msg)
            self.joy2_pub.publish(msg)

        self.last_controller = controller


if __name__ == '__main__':
    rospy.init_node('switchJoyNode')
    sj = SwitchJoy(strToJoyKind(rospy.get_param('~joy_kind', 'default')))
    rospy.spin()


