#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

from sensor_msgs.msg import Joy

from ROS_StateHandler import ROS_Handler

# from PyQt4 import QtGui, QtCore
from math import pi

import tf

Step = dict( 
	x = 0.1, 
	y = 0.1, 
	z = 0.1, 
	yaw = pi/20)

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

	def X(self, scale):
		self.change_set_point('x', scale)

	def Y(self, scale):
		self.change_set_point('y', scale)

	def Z(self, scale):
		self.change_set_point('z', scale)

	def Yaw(self, scale):
		self.change_set_point('yaw', scale)
		quaternion = tf.transformations.quaternion_from_euler(self.quadrotor.position.yaw, self.quadrotor.position.pitch, self.quadrotor.position.roll, 'rzyx')
		self.quadrotor.orientation.x = quaternion[0]
		self.quadrotor.orientation.y = quaternion[1]
		self.quadrotor.orientation.z = quaternion[2]
		self.quadrotor.orientation.w = quaternion[3]

	def change_set_point(self, direction, scale):
		setattr(self.quadrotor.position, direction, 
			getattr(self.quadrotor.position, direction) + scale * Step[direction] )


	def RecieveJoy(self, data):
		for command, message_data in self.MAP.items():
			for key, index in message_data.items():
				if getattr(data, key)[index]:
					getattr(self, command)( getattr(data, key)[index] )

		
	"""
	def keyPressEvent(self, event):
		key = event.key()
		if key == self.MAP['Takeoff']:
			self.TakeOff()
		elif key == self.MAP['Land']:
			self.Land()
		elif key == self.MAP['Emergency']:
			self.Reset()
		elif key == self.MAP['RollLeft']:
			self.quadrotor.position.x += StepX
			print "+X"
		elif key == self.MAP['RollRight']:
			self.quadrotor.position.x -= StepX
			print "-X"
		elif key == self.MAP['PitchForward']:
			self.quadrotor.position.y -= StepY
			print "+Y"
		elif key == self.MAP['PitchBackward']:
			self.quadrotor.position.y += StepY
			print "-Y"
		elif key == self.MAP['IncreaseAltitude']:
			self.quadrotor.position.z += StepZ
			print "+z"
		elif key == self.MAP['DecreaseAltitude']:
			self.quadrotor.position.z -= StepZ
			print "-z"
		elif key == self.MAP['YawAntiClockwise']:
			self.quadrotor.position.yaw += StepYaw
			print "+Yaw"
			quaternion = tf.transformations.quaternion_from_euler(self.quadrotor.position.yaw, self.quadrotor.position.pitch, self.quadrotor.position.roll, 'rzyx')
			self.quadrotor.orientation.x = quaternion[0]
			self.quadrotor.orientation.y = quaternion[1]
			self.quadrotor.orientation.z = quaternion[2]
			self.quadrotor.orientation.w = quaternion[3]
 		elif key == self.MAP['YawClockwise']:
 			print "-Yaw"
			self.quadrotor.position.yaw -= StepYaw
			quaternion = tf.transformations.quaternion_from_euler(self.quadrotor.position.yaw, self.quadrotor.position.pitch, self.quadrotor.position.roll, 'rzyx')
			self.quadrotor.orientation.x = quaternion[0]
			self.quadrotor.orientation.y = quaternion[1]
			self.quadrotor.orientation.z = quaternion[2]
			self.quadrotor.orientation.w = quaternion[3]
	"""
		
def main():  
	rospy.init_node('StateHandler', anonymous = True)

	joy = JoystickController()

	rospy.spin()
		


    

if __name__ == "__main__": main()