#!/usr/bin/env python

"""
test_actuator.py is a test script for reading in 
text from from a speech recognizer script and map 
that text to control the Baxter simulator
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math

from std_msgs.msg import String
from std_msgs.msg import Float64

##
import argparse
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

def speechCb(msg):

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_velocity = limb.joint_velocity(joint_name)
        joint_command = {joint_name: current_velocity + delta}
        limb.set_joint_velocities(joint_command)
    
    if msg.data.find("move left") > -1:
	set_j(left, lj[0], 0.2)
	print("move left")

    if msg.data.find("move right") > -1:
    	set_j(left, lj[0], -0.2)
	print("move right")

def main():

    #Baxter
    print("Initializing node... ")
    rospy.init_node("actuator")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    #Voice
    rospy.Subscriber('basic_recognizer/output', String, speechCb)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == '__main__':
    main()
