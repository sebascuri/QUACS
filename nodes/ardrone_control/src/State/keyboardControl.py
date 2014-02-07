#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy
import sys
from ROS_StateHandler import ROS_Handler

from PyQt4 import QtGui, QtCore
from math import pi

import tf


Step = dict( 
	x = 0.1, 
	y = 0.1, 
	z = 0.1, 
	yaw = pi/20)

class KeyboardController(ROS_Handler, QtGui.QMainWindow, object):
	"""docstring for KeyboardController
	MAP is a dict from ROS_Handler method names to a list of keyboard that should call that method. 

	When a key is pressed the method keyPressEvent calls the method 
	and passes as a parameter the position of the list in which it appears 
	so as to increase or decreasethe set point 

	At the same time it inits a simple QTGUI application to handle keyPress events. 
	"""

	MAP = dict( 
		X 					= [ QtCore.Qt.Key_Down, QtCore.Qt.Key_Up ],
		Y 					= [ QtCore.Qt.Key_Right, QtCore.Qt.Key_Left ],
		Z 					= [ QtCore.Qt.Key_S, QtCore.Qt.Key_W ],
		Yaw 				= [ QtCore.Qt.Key_A, QtCore.Qt.Key_D ],
		TakeOff          	= [ QtCore.Qt.Key_Return ],
		Land             	= [ QtCore.Qt.Key_Backspace ],
		Reset        		= [ QtCore.Qt.Key_Space ],
		ControlOff 			= [ QtCore.Qt.Key_O ],
		GoToGoal 			= [ QtCore.Qt.Key_G ]
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
		for command, key_code in self.MAP.items():
			if key in key_code:
				getattr(self, command)( (key_code.index(key) - 0.5) * 2 )
		
def main():  
	import sys  
	rospy.init_node('StateHandler', anonymous = True)


	app = QtGui.QApplication(sys.argv)
	keyb = KeyboardController()

	# executes the QT application
	status = app.exec_()

	rospy.spin()

	# sys.exit(status)
		   

if __name__ == "__main__": main()