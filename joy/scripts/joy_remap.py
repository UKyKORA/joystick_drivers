#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import traceback
from rover_msgs.msg import SingleMotorSetting
from sensor_msgs.msg import Joy

class JoyRemap(object):
    def __init__(self):
        
        self.motors = [[0,0.0],[1,0.0],[2,0.0],[3,0.0]]
        
        self.pub = rospy.Publisher(
            "/drive_setting",
            SingleMotorSetting, 
            queue_size=1)
            
        self.sub = rospy.Subscriber(
            "joy",
            Joy, 
            self.callback,
            queue_size=1)
            
        

    def callback(self, in_msg):
        out_msg = SingleMotorSetting()
        
        max_throttle = (in_msg.axes[3] + 1.0)/2.0
        drive = in_msg.axes[1]
        steer = in_msg.axes[0] * -1.0
        
        print(in_msg.axes[3])
        
        if abs(drive) > abs(steer):
            self.motors[0][1] = (100.0 * max_throttle) * drive
            self.motors[1][1] = (100.0 * max_throttle) * drive
            self.motors[2][1] = (100.0 * max_throttle) * drive
            self.motors[3][1] = (100.0 * max_throttle) * drive
        else:
            self.motors[0][1] = -1.0 * (100.0 * max_throttle) * steer
            self.motors[3][1] = (100.0 * max_throttle) * steer
            self.motors[1][1] = -1.0 * (100.0 * max_throttle) * steer
            self.motors[2][1] = (100.0 * max_throttle) * steer
        
        print(self.motors)
        
        for i in self.motors:
            out_msg.motor_id = i[0]
            out_msg.motor_value = int(i[1])
            self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node("joy_remap")
    n = JoyRemap()
    rospy.spin()
