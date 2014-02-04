#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

from Quadrotor import Quadrotor

import tf



class ControllerState(object):
	"""docstring for ControllerState"""
	def __init__(self, state = 0):
		super(ControllerState, self).__init__()
		self._STATES = { 0: 'Go-to-Goal', 1: 'Avoid-Obstacles', 2: 'Sliding-Mode' }
		self.state = state 

	def __str__(self):
		return str(self._state)

	def __eq__(self, data):
		if type(data) == str:
			return str(self._state) == data
		elif type(data) == int:
			return str(self._state) == self._STATES[data]	

	@property 
	def state(self):
		return self._state 
	@state.setter
	def state(self, state):
		if (state in self._STATES.values()) or (state in self._STATES.keys()):
			if type(state) == int:
				self._state = self._STATES[state]
			elif type(state) == float:
				self._state = self._STATES[int(state)]
			else:
				self._state = state
		else:
			self._state = self._STATES[0]
			print 'State not recognized, setting it to Unkown'

	@state.deleter
	def state(self):
		del self._state

class ROS_Handler(object):
	"""docstring for ROS_Handler"""
	def __init__(self, **kwargs):
		super(ROS_Handler, self).__init__()
		
		self.land = rospy.Publisher('/ardrone/land', Empty)
		self.takeoff = rospy.Publisher('/ardrone/takeoff', Empty, latch = True)
		self.reset = rospy.Publisher('/ardrone/reset', Empty)

		self.quadrotor = Quadrotor()

		# self.controller = ControllerState()

		self.name = kwargs.get('name', "/goal")

		rospy.Subscriber('/ardrone/navdata', Navdata, callback = self.RecieveNavdata)
		self.trajectory = rospy.Publisher('/ardrone/trajectory', Odometry)
		self.goal_tf = tf.TransformBroadcaster()

		rospy.Timer(rospy.Duration( kwargs.get('Command_Time', 1) ), self.Talk, oneshot=False)

	def RecieveNavdata( self, data ):
		self.quadrotor.set_state(data.state)

	def Talk( self, time ):
		""" Publishes the Goal Point for the Controller """

		msgs = Odometry( )
		msgs.header.stamp = rospy.Time.now()
		msgs.header.frame_id = "/nav"
		msgs.child_frame_id = self.name 

		# position
		msgs.pose.pose.position.x = self.quadrotor.position.x
		msgs.pose.pose.position.y = self.quadrotor.position.y
		msgs.pose.pose.position.z = self.quadrotor.position.z

		# orientation
		msgs.pose.pose.orientation.x = self.quadrotor.orientation.x
		msgs.pose.pose.orientation.y = self.quadrotor.orientation.y
		msgs.pose.pose.orientation.z = self.quadrotor.orientation.z
		msgs.pose.pose.orientation.w = self.quadrotor.orientation.w
		
		# linear velocity
		msgs.twist.twist.linear.x = self.quadrotor.velocity.x
		msgs.twist.twist.linear.y = self.quadrotor.velocity.y
		msgs.twist.twist.linear.z = self.quadrotor.velocity.z

		# angular velocity
		msgs.twist.twist.angular.x = self.quadrotor.velocity.roll
		msgs.twist.twist.angular.y = self.quadrotor.velocity.pitch
		msgs.twist.twist.angular.z = self.quadrotor.velocity.yaw

		self.trajectory.publish(msgs)

		self.goal_tf.sendTransform( (msgs.pose.pose.position.x, msgs.pose.pose.position.y, msgs.pose.pose.position.z) , 
            (msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y, msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w), 
            msgs.header.stamp, 
            msgs.child_frame_id, 
            msgs.header.frame_id)

	def TakeOff(self, *args):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.quadrotor.state == 'Landed'):
			self.takeoff.publish()
			print "Taking Off!"
		else:
			print "Drone is not landed"

	def Reset(self, *args):
		self.reset.publish()
		print "Reset"

	def Land(self, *args):
		self.land.publish()
		print "Landing"


def main():
	rospy.init_node('StateHandler', anonymous = True)
	ros_handler = ROS_Handler(Command_Time = 1)
	

	rospy.spin()
		


    

if __name__ == "__main__": main()