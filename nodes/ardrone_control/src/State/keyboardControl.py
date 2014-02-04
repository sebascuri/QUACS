#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy
import sys
from ROS_StateHandler import ROS_Handler

from PyQt4 import QtGui, QtCore
from math import pi

import tf


StepX = 0.1
StepY = 0.1
StepZ = 0.1
StepYaw = pi/10

class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key_Right
	PitchBackward    = QtCore.Qt.Key_Left
	RollLeft         = QtCore.Qt.Key_Up
	RollRight        = QtCore.Qt.Key_Down
	YawLeft          = QtCore.Qt.Key_A
	YawRight         = QtCore.Qt.Key_D
	IncreaseAltitude = QtCore.Qt.Key_W
	DecreaseAltitude = QtCore.Qt.Key_S
	Takeoff          = QtCore.Qt.Key_Return
	Land             = QtCore.Qt.Key_Backspace
	Emergency        = QtCore.Qt.Key_Space

class KeyboardController(ROS_Handler, QtGui.QMainWindow, object):
	"""docstring for KeyboardController"""
	MAP = dict(
		PitchForward     = QtCore.Qt.Key_Right,
		PitchBackward    = QtCore.Qt.Key_Left,
		RollLeft         = QtCore.Qt.Key_Up,
		RollRight        = QtCore.Qt.Key_Down,
		YawAntiClockwise = QtCore.Qt.Key_A,
		YawClockwise     = QtCore.Qt.Key_D,
		IncreaseAltitude = QtCore.Qt.Key_W,
		DecreaseAltitude = QtCore.Qt.Key_S,
		Takeoff          = QtCore.Qt.Key_Return,
		Land             = QtCore.Qt.Key_Backspace,
		Emergency        = QtCore.Qt.Key_Space,
		)
	def __init__(self, **kwargs):
		super(KeyboardController, self).__init__(**kwargs)
		self.initUI()

	def initUI( self ):
		self.setGeometry(300,300,250,150)
		self.setWindowTitle('AR.Drone KeyboardController')
		self.show()
		
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

		
def main():  
	import sys  
	rospy.init_node('StateHandler', anonymous = True)


	app = QtGui.QApplication(sys.argv)
	keyb = KeyboardController()

	# executes the QT application
	status = app.exec_()

	rospy.spin()

	sys.exit(status)
		


    

if __name__ == "__main__": main()