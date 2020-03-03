#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import traceback
from sensor_msgs.msg import Joy
from geometry_msgs import Twist

class JoyRemap(object):
    def __init__(self):
        self.lin_max = rospy.get_param("/drive/lin_max")
        self.ang_max = rospy.get_param("/drive/ang_max")
        self.pub = rospy.Publisher(
            "/drive_setting",
            Twist,
            queue_size=1)

        self.sub = rospy.Subscriber(
            "joy",
            Joy,
            self.callback,
            queue_size=1)

    def callback(self, in_msg):

        out_msg = Twist()

        max_throttle = (in_msg.axes[3] + 1.0)/2.0 # 0 - 1 throttle control

        drive = in_msg.axes[1]         # y axis of joy -1 to 1
        steer = in_msg.axes[0] * -1.0  # x axis of joy -1 to 1

        out_msg.linear.y = drive * self.lin_max * max_throttle  # apply throttle (0 - 100% speed) and scale to max linear velocity
        out_msg.angular.x = steer * self.ang_max * max_throttle # apply throttle (0 - 100% speed) and scale to max angular velocity

        self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node("joy_twist_remap")
    n = JoyRemap()
    rospy.spin()
