#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

from sensor_msgs.msg import Joy #dont forget to rosrun joy joy_node !

from ROS_StateHandler import ROS_Handler

from math import pi

import tf


class JoystickController(ROS_Handler, object):
	"""docstring for JoystickController:
	MAP is a dict from ROS_Handler method names to a dict with Joy message id. 
	The joy message id contains the field inside joy and the index of the array of such field. 

	When a message is recieved the method RecieveJoy looks to see if such position is non-empty, 
	if so it calls the method name and passes as a parameter the value it reads. 

	For buttons the values ar boolean, but for axes are floats from -1 to 1.   
	"""
	MAP = dict(
		X     			= {'axes' : 3}, #Right Analog Up-Down
		Y    			= {'axes' : 2}, #Right Analog Left-Right
		Z         		= {'axes' : 1}, #Left Analog Up-Down
		Yaw 			= {'axes' : 0}, #Left Analog Left-Right
		TakeOff         = {'buttons' : 14}, #Cross Button
		Land            = {'buttons' : 15}, #Square Button
		Reset       	= {'buttons' : 12}, #Triange Button
		ControlOff 		= {'buttons' : 13}, #Circle Button
		ControlOn 		= {'buttons' : 3 }, #Start Button
		ChirpX			= {'buttons' : 5 }, #Right arrow
		ChirpY			= {'buttons' : 7 }, #Left arrow
		ChirpZ			= {'buttons' : 4 }, #Top arrow
		ChirpYaw		= {'buttons' : 6 } #Bottom arrow
		) 

	def __init__(self, **kwargs):
		super(JoystickController, self).__init__(**kwargs)
		rospy.Subscriber('/joy', Joy, callback = self.RecieveJoy)

	def RecieveJoy(self, data):
		for command, message_data in self.MAP.items(): 
			for key, index in message_data.items(): 
				if getattr(data, key)[index]: 
					getattr(self, command)( getattr(data, key)[index] )

		
def main():  
	rospy.init_node('StateHandler', anonymous = True)

	joy = JoystickController()

	rospy.spin()
		


    

if __name__ == "__main__": main()