#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

from sensor_msgs.msg import Joy

from ROS_StateHandler import ROS_Handler

# from PyQt4 import QtGui, QtCore
from math import pi

import tf


class JoystickController(ROS_Handler, object):
	"""docstring for JoystickController"""
	MAP = dict(
		X     			= {'axes' : 3}, #Right Analog Up-Down
		Y    			= {'axes' : 2}, #Right Analog Left-Right
		Z         		= {'axes' : 1}, #Left Analog Up-Down
		Yaw 			= {'axes' : 0}, #Left Analog Left-Right
		TakeOff         = {'buttons' : 14}, #Cross Button
		Land            = {'buttons' : 15}, #Square Button
		Reset       	= {'buttons' : 11}, #R1 Button
		) 
	def __init__(self, **kwargs):
		super(JoystickController, self).__init__(**kwargs)
		rospy.Subscriber('/joy', Joy, callback = self.RecieveJoy)

	def RecieveJoy(self, data):
		for command, message_data in self.MAP.items():
			for key, index in message_data.items():
				if getattr(data, key)[index]: #if non-zero
					getattr(self, command)( getattr(data, key)[index] )
		
def main():  
	rospy.init_node('StateHandler', anonymous = True)

	joy = JoystickController()

	rospy.spin()
		


    

if __name__ == "__main__": main()