#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_sensorfusion')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry 
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

import tf


def ReceiveSonar(sonar):
	pass

def ReceiveNavdata(navdata):
	print type(navdata.state), navdata.state
	print type(navdata.batteryPercent), navdata.batteryPercent
	print type(navdata.rotX), navdata.rotX
	print type(navdata.rotY), navdata.rotY
	print type(navdata.rotZ), navdata.rotZ
	print type(navdata.vx), navdata.vx # mm/s
	print type(navdata.vy), navdata.vy # mm/s
	print type(navdata.vz), navdata.vz # mm/s

	print type(navdata.ax), navdata.ax # g
	print type(navdata.ay), navdata.ay # g
	print type(navdata.az), navdata.az # g

	print type(navdata.tm), navdata.tm # micro seconds

	print type(navdata)

def ReceiveImu(ImuRaw):

	print type(ImuRaw.orientation.x), ImuRaw.orientation.x
	print type(ImuRaw.orientation.y), ImuRaw.orientation.y
	print type(ImuRaw.orientation.z), ImuRaw.orientation.z
	print type(ImuRaw.orientation.w), ImuRaw.orientation.w

	quaternion = (ImuRaw.orientation.x, ImuRaw.orientation.y, ImuRaw.orientation.z, ImuRaw.orientation.w)
	euler = tf.transformations.euler_from_quaternion( quaternion )
	print "Euler", euler
	#print "Euler", type(euler)

	print type(ImuRaw.orientation_covariance), ImuRaw.orientation_covariance


	print type(ImuRaw.angular_velocity.x), ImuRaw.angular_velocity.x
	print type(ImuRaw.angular_velocity.y), ImuRaw.angular_velocity.y
	print type(ImuRaw.angular_velocity.z), ImuRaw.angular_velocity.z

	print type(ImuRaw.angular_velocity_covariance), ImuRaw.angular_velocity_covariance


	print type(ImuRaw.linear_acceleration.x), ImuRaw.linear_acceleration.x
	print type(ImuRaw.linear_acceleration.y), ImuRaw.linear_acceleration.y
	print type(ImuRaw.linear_acceleration.z), ImuRaw.linear_acceleration.z

	print type(ImuRaw.linear_acceleration_covariance), ImuRaw.linear_acceleration_covariance




def Listener(data, function):
	function(data)


def main():
	print "Starting Node"
	rospy.init_node('SensorFusion_Test', anonymous = True)
	print "Node Initied"
	

	# rospy.Subscriber('ardrone/navdata',Navdata, callback = Listener, callback_args = ReceiveNavdata) 

	# rospy.Subscriber('ardrone/imu', Imu, callback = Listener, callback_args = ReceiveImu)

	# rospy.Subscriber('ardrone/sonar_height', Imu, callback = Listener, callback_args = ReceiveImu)

	rospy.spin()
	

if __name__ == "__main__": main()