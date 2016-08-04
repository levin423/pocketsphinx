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

	def cmd(cmd_wrd):
		global side
		global joint
		if cmd_wrd[0].find("left") > -1 or cmd_wrd[0].find("right") > -1:
			
			if cmd_wrd[0].find("left") > -1:
				side = left
			elif cmd_wrd[0].find("right") > -1:
				side = right

			if cmd_wrd[1].find("shoulder") > -1:
				joint =  "shoulder"
			elif cmd_wrd[1].find("elbow") > -1:
				joint =  "elbow"
			elif cmd_wrd[1].find("wrist") > -1:
				joint =  "wrist"
			print(side, joint, "selected") 

		elif cmd_wrd[0].find("move") > -1 or cmd_wrd[0].find("shift") > -1:
			'''try: 
				joint
			except NameError: 
				print ("Limb/Joint Undefined")
			else:'''
			print ("Doing work")
			if side == left:
				if cmd_wrd[1].find("left") > -1 or cmd_wrd[1].find("right") > -1:
					if joint == "shoulder":
						joint_dof = lj[0]
					elif joint == "elbow":
						joint_dof = lj[2]
					elif joint == "wrist":
						joint_dof = lj[4]
					
				elif cmd_wrd[1].find("up") > -1 or cmd_wrd[1].find("down") > -1:
					if joint == "shoulder":
						joint_dof = lj[1]
					elif joint == "elbow":
						joint_dof = lj[2]
					elif joint == "wrist":
						joint_dof = lj[5]

			elif side == right:
				if cmd_wrd[1].find("left") > -1 or cmd_wrd[1].find("right") > -1:
					if joint == "shoulder":
						joint_dof = rj[0]
					elif joint == "elbow":
						joint_dof = rj[2]
					elif joint == "wrist":
						joint_dof = rj[4]
					
				elif cmd_wrd[1].find("up") > -1 or cmd_wrd[1].find("down") > -1:
					if joint == "shoulder":
						joint_dof = rj[1]
					elif joint == "elbow":
						joint_dof = rj[2]
					elif joint == "wrist":
						joint_dof = rj[5]

			if cmd_wrd[1].find("left") > -1 or cmd_wrd[1].find("up") > -1:
				velocity = 0.2
			elif cmd_wrd[1].find("right") > -1 or cmd_wrd[1].find("down") > -1:
				velocity = -0.2
			set_j(side, joint_dof, velocity)
			print(side)
			print(joint)
			print(joint_dof)
				

		elif cmd_wrd[0].find("stop") > -1:
			for i in range(0,6):
				set_j(left, lj[i], 0)
				set_j(right, rj[i], 0)
			print("stopped")
				

	cmd_wrd = msg.data.split()
	cmd(cmd_wrd)

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
    rospy.Subscriber('advanced_recognizer/output', String, speechCb)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
    	r.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
