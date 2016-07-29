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

    rospy.logwarn(msg.data)
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, velocity):
	current_velocity = limb.joint_velocity(joint_name)
	joint_command = {joint_name: velocity}
        limb.set_joint_velocities(joint_command)   

    cmd_wrd = msg.data.split()
    print("1")
    if cmd_wrd[0].find("move") > -1 or cmd_wrd[0].find("shift") > -1:
	print("2")
	#move left
	if cmd_wrd[1].find("left") > -1:
	    print("3")
	    #move left shoulder
	    if cmd_wrd[2].find("shoulder") > -1:
		side = left
		print("4")
		#move left shoulder left
		if cmd_wrd[3].find("left") > -1:
		    joint = lj[0]
		    #move left shoulder left faster
		    if cmd_wrd[4].find("faster") > -1:
			set_j(side, joint, abs(current_velocity) + .1)
		        print("Moving left shoulder left faster")
		    #move left shoulder left faster
		    elif cmd_wrd[4].find("slower") > -1:
			if abs(current_velocity) - .1 <= 0:
			   set_j(side, joint, 0)
			   print("Left shoulder stopped for speed")
			else:
			    set_j(side, joint, abs(current_velocity) - .1)
			    print("Moving left shoulder left slower")
		    #move left shoulder left (no modifier)
		    else:
		    	print("5")
			set_j(left, lj[0], .2)
			print("Moving left shoulder left")

		#move left shoulder right			
		elif cmd_wrd[3].find("right") > -1:
		    joint = lj[0]
		    #move left shoulder right faster
		    if cmd_wrd[4].find("faster") > -1:
			set_j(side, joint, -abs(current_velocity) - .1)
		        print("Moving left shoulder right faster")
		    #move left shoulder right slower
		    elif cmd_wrd[4].find("slower") > -1:
			if -abs(current_velocity) + .1 >= 0:
			   set_j(side, joint, 0)
			   print("Left shoulder stopped for speed")
			else:
			    set_j(side, joint, -abs(current_velocity) + .1)
			    print("Moving left shoulder right slower")
		    #move left shoulder right (no modifier)
		    else:
			set_j(side, joint, -.2)
			print("Moving left shoulder right")

		#move left shoulder up
		elif cmd_wrd[3].find("up") > -1 or cmd_wrd[3].find("higher") > -1:
		    joint = lj[1]
		    #move left shoulder up faster
		    if cmd_wrd[4].find("faster") > -1:
			set_j(side, joint, abs(current_velocity) + .1)
		        print("Moving left shoulder up faster")
		    #move left shoulder up slower
		    elif cmd_wrd[4].find("slower") > -1:
			if abs(current_velocity) - .1 >= 0:
			   set_j(side, joint, 0)
			   print("Left shoulder stopped for speed")
			else:
			    set_j(side, joint, abs(current_velocity) - .1)
			    print("Moving left shoulder up slower")
		    #move left shoulder up (no modifier)
		    else:
			set_j(side, joint, .2)
			print("Moving left shoulder up")

		#move left shoulder down
		elif cmd_wrd[3].find("down") > -1 or cmd_wrd[3].find("lower") > -1:
		    joint = lj[1]
		    #move left shoulder down faster
		    if cmd_wrd[4].find("faster") > -1:
			set_j(side, joint, -abs(current_velocity) - .1)
		        print("Moving left shoulder down faster")
		    #move left shoulder down slower
		    elif cmd_wrd[4].find("slower") > -1:
			if -abs(current_velocity) + .1 >= 0:
			   set_j(side, joint, 0)
			   print("Left shoulder stopped for speed")
			else:
			    set_j(side, joint, -abs(current_velocity) + .1)
			    print("Moving left shoulder down slower")
		    #move left shoulder down (no modifier)
		    else:
			set_j(side, joint, -.2)
			print("Moving left shoulder down")

	    elif cmd_wrd[2].find("elbow") > -1:
		print("move left elbow ... under construction")
	    elif cmd_wrd[2].find("wrist") > -1:
		print("move left wrist ... under construction")
	    #move current joint left
	    else:
		try: joint
		except: print("Limb/Joint is undefined")
		else:
		    #move left faster
		    if cmd_wrd[4].find("faster") > -1:
			set_j(side, joint, abs(current_velocity) + .1)
		        print("Moving left faster")
		    #move left faster
		    elif cmd_wrd[4].find("slower") > -1:
			if abs(current_velocity) - .1 <= 0:
			   set_j(side, joint, 0)
			   print("Motion stopped for speed")
			else:
			    set_j(side, joint, abs(current_velocity) - .1)
			    print("Moving left slower")
		    #move left (no modifier)
		    else:
			set_j(side, joint, .2)
			print("Moving left")


	elif cmd_wrd[1].find("right") > -1:
    	    print("move right ... under construction")
    elif cmd_wrd[0].find("stop") > -1:
	set_j(left, lj[:], 0)
	set_j(right, rj[:], 0)
	print("stopped")

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

    #rospy.spin()

if __name__ == '__main__':
    main()
