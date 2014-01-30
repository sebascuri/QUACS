#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_controller')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

import Controller 

from math import pi, cos, sin 
import tf 

# Some Constants
Command_Time = 0.01;
g = 9.81;

class DroneController(object):
	"""docstring for DroneController"""
	def __init__(self):
		super(DroneController, self).__init__()
		self.position_control = dict( x = Controller.PID_Controller( Ts = Command_Time, Kp = 1.0, Ki = 0.0, Kd = 0.0 ), 
			y = Controller.PID_Controller( Ts = Command_Time, Kp = 1.0, Ki = 0.0, Kd = 0.0 ), 
			z = Controller.PID_Controller( Ts = Command_Time, Kp = 1.0, Ki = 0.0, Kd = 0.0 ) )

		self.orientation_control = dict( z = Controller.PID_Controller( Ts = Command_Time, Kp = 1.0, Ki = 0.0, Kd = 0.0 ) )
		self.yaw = 0
	

class ROS_Handler(DroneController, object):
	"""docstring for ROS_Handler"""
	def __init__(self, **kwargs):
		super(ROS_Handler, self).__init__()

		
		rospy.Timer(rospy.Duration( Command_Time ), self.Actuate, oneshot=False)
		
		rospy.Subscriber('ardrone/sensorfusion/navdata', Odometry, callback = self.RecieveOdometry, callback_args = 'set_input' )
		rospy.Subscriber('ardrone/trajectory', Odometry, callback = self.RecieveOdometry, callback_args = 'change_set_point' )

		self.command_velocity = rospy.Publisher('cmd_vel', Twist)

		self.angles_map = dict(x = 0, y = 1 , z = 2)
		


	def RecieveOdometry( self, data , method):
		for key in self.position_control.keys():
			getattr(self.position_control[key], method)( position = getattr(data.pose.pose.position, key), velocity = getattr(data.twist.twist.linear, key ) )

		angles = tf.transformations.euler_from_quaternion( [data.pose.pose.orientation.x , data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] )
		for key in self.orientation_control.keys():
			getattr(self.orientation_control[key], method) ( position = angles[ self.angles_map[key] ] , velocity = getattr(data.twist.twist.angular, key ) )

		if method == 'set_input':
			self.yaw = self.angles[ self.angles_map['z'] ]

	def Actuate(self, time):
		twist = Twist()
		for key in self.position_control.keys():
			setattr(twist.linear, key, self.position_control[key].get_output( ))


		for key in self.orientation_control.keys( ):
			setattr(twist.angular, key, self.orientation_control[key].get_output( ))


		aux_x = twist.linear.x * cos(self.yaw) + twist.linear.y * sin(self.yaw)
		aux_y = - twist.linear.x * sin(self.yaw) + twist.linear.y * cos(self.yaw)
		twist.linear.x = aux_x
		twist.linear.y = aux_y
		self.command_velocity.publish(twist)




def main():    
	rospy.init_node('Controller', anonymous = True)
	ros_handler = ROS_Handler()
	

	rospy.spin()

    

if __name__ == "__main__": main()