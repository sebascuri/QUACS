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
from math import pi


Step = dict( 
	x = 0.1, 
	y = 0.1, 
	z = 0.1, 
	yaw = pi/20)


class ROS_Handler(object):
	"""docstring for ROS_Handler:
	Subscribed to:
		ardrone/navdata -> reads state of the ardrone

	Publishes to:
		ardrone/land
		ardrone/takeoff
		ardrone/reset
		controllerstate -> communicates to the controller its state

		ardrone/trajectory -> Prints in an Odometry msg the desired trajectory 
			It is called by a Timer 
			trajectory data is taken from a self.quadrotor object instance 

	"""
	def __init__(self, **kwargs):
		super(ROS_Handler, self).__init__()
		
		self.land = rospy.Publisher('/ardrone/land', Empty, latch = True)
		self.takeoff = rospy.Publisher('/ardrone/takeoff', Empty, latch = True)
		self.reset = rospy.Publisher('/ardrone/reset', Empty, latch = True)

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

	def X(self, scale):
		self.change_set_point('x', scale)

	def Y(self, scale):
		self.change_set_point('y', scale)

	def Z(self, scale):
		self.change_set_point('z', scale)

	def Yaw(self, scale):
		self.change_set_point('yaw', scale) 
		# Order is important: first change set point in yaw, then update quaternions.
		if self.quadrotor.position.yaw >=  pi :
			self.quadrotor.position.yaw -= 2 * pi;
		if self.quadrotor.position.yaw < -pi:
			self.quadrotor.position.yaw += 2 * pi;

		quaternion = tf.transformations.quaternion_from_euler(self.quadrotor.position.yaw, self.quadrotor.position.pitch, self.quadrotor.position.roll, 'rzyx')
		
		self.quadrotor.orientation.x = quaternion[0]
		self.quadrotor.orientation.y = quaternion[1]
		self.quadrotor.orientation.z = quaternion[2]
		self.quadrotor.orientation.w = quaternion[3]

	def change_set_point(self, direction, scale):
		# it changes an attribute from the self.quadrotor.position by a fraction of a Step.
		setattr(self.quadrotor.position, direction, 
			getattr(self.quadrotor.position, direction) + scale * Step[direction] )

def main():
	rospy.init_node('StateHandler', anonymous = True)
	ros_handler = ROS_Handler(Command_Time = 1)
	

	rospy.spin()
		


    

if __name__ == "__main__": main()