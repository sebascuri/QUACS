#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy;

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import NavSatFix, Range
from nav_msgs.msg import Odometry 

from ROS import ROS_Object 
from Sensors import GPS 
# from SensorFusion import IMU_Kalman, IMU_Magdwick, IMU_Mahoney

from math import pi, cos, sin 
import tf 

class ROS_GPS(ROS_Object, object):
	"""docstring for ROS_IMU"""
	def __init__(self, **kwargs):
		super(ROS_GPS, self).__init__()

		self.gps = GPS( )

		# self.imu = IMU_Magdwick( Ts = 0.01 )
		# self.imu = IMU_Mahoney( Ts = 0.01 )
		self.publisher = dict(
            state = rospy.Publisher('/ardrone/gps', Odometry)
            )

		self.subscriber = dict(
            raw_gps = rospy.Subscriber('/fix', NavSatFix, callback = self.ReceiveGPS ),
            )
		

	def ReceiveGPS(self, gps_raw):
		if not self.gps.calibrated:
			self.gps.set_zero(gps_raw)
			self.gps.calibrated = True 


		self.gps.measure(gps_raw)

		self.Talk()

	def Talk( self ):

		msg = Odometry()
		msg.header.stamp = rospy.Time.now()

		msg.pose.pose.position.x = self.gps.x
		msg.pose.pose.position.y = self.gps.y

		print self.gps.x, self.gps.y, self.gps.gps_zero['x'], self.gps.gps_zero['y']

		self.publisher['state'].publish(msg) 

def main():

	rospy.init_node('GPS_Test', anonymous = True)
	node = ROS_GPS()

	rospy.spin()

if __name__ == "__main__": main()
