#!/usr/bin/env python

"""
test_actuator.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.

For basic arm7dof 1dof velocity voice control
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math

from std_msgs.msg import String
from std_msgs.msg import Float64

class test_actuator:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.msg = Float64()

        # publish to cmd_vel, subscribe to speech output
        #self.pub_ = rospy.Publisher('cmd_vel', Float64)
	self.pub_ = rospy.Publisher('/arm7dof/joint1_velocity_controller/command', Float64)
        rospy.Subscriber('basic_recognizer/output', String, self.speechCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)
            r.sleep()
        
    def speechCb(self, msg):
        

        if msg.data.find("move left") > -1:
                self.msg = .5

	if msg.data.find("move right") > -1:
                self.msg = -0.5
        
        rospy.loginfo(msg.data)
        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        float64 = Float64()
        self.pub_.publish(float64)

if __name__=="__main__":
    rospy.init_node('test_actuator')
    try:
        test_actuator()
    except:
        pass

