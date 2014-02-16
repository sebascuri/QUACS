#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry 

from ROS import ROS_Object 
from Sensors import IMU 
from SensorFusion import IMU_Kalman, IMU_Magdwick, IMU_Mahoney

from math import pi, cos, sin 
import tf 

class ROS_IMU(ROS_Object, object):
	"""docstring for ROS_IMU"""
	def __init__(self, **kwargs):
		super(ROS_IMU, self).__init__()
		Kp = rospy.get_param('Mahoney/Kp')
		print Kp


		self.imu = IMU_Mahoney(Ts = 0.01)
		self.a = self.imu.quaternion

		# self.imu = IMU_Magdwick( Ts = 0.01 )
		# self.imu = IMU_Mahoney( Ts = 0.01 )

		self.subscriber = dict(
            raw_imu = rospy.Subscriber('/ardrone/imu', Imu, callback = self.ReceiveImu ),
            )
		self.publisher = dict(
            state = rospy.Publisher('/ardrone/kalman_imu', Odometry)
            )

	def ReceiveImu(self, imu_raw):
		self.imu.measure(imu_raw)
		self.imu.predict()
		self.imu.correct()

		self.Talk()

		#print self.a.z, self.imu.quaternion.z

	def Talk( self ):
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		quaternion = self.imu.get_quaternion()

		msg.pose.pose.orientation.x = quaternion['x']
		msg.pose.pose.orientation.y = quaternion['y']
		msg.pose.pose.orientation.z = quaternion['z']
		msg.pose.pose.orientation.w = quaternion['w']

		self.publisher['state'].publish(msg) 

def main():

	rospy.init_node('Imu_Test', anonymous = True)
	node = ROS_IMU()

	rospy.spin()

if __name__ == "__main__": main()