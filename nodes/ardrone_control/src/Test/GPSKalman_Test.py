#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy;

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import NavSatFix, Range
from nav_msgs.msg import Odometry 
from ardrone_autonomy.msg import Navdata

from ROS import ROS_Object 
from SensorFusion import GPS_Filter 
# from SensorFusion import IMU_Kalman, IMU_Magdwick, IMU_Mahoney

from math import pi, cos, sin 
import tf 

class ROS_GPS(ROS_Object, object):
	"""docstring for ROS_IMU"""
	def __init__(self, **kwargs):
		super(ROS_GPS, self).__init__()

		self.gps = GPS_Filter( Ts = 0.005 )

		self.publisher = dict(
            state = rospy.Publisher('/ardrone/gps', Odometry)
            )

		self.subscriber = dict(
            raw_gps = rospy.Subscriber('/fix', NavSatFix, callback = self.ReceiveGPS ),
            navdata = rospy.Subscriber('/ardrone/navdata', Navdata, callback = self.ReceiveNavdata)
            )
		
	def ReceiveNavdata(self, navdata):
		self.gps.measure_navdata(navdata)

		#dummy input
		self.gps.position.set_properties( yaw = navdata.rotZ * pi/180 )

		self.gps.predict()

		self.Talk()
	def ReceiveGPS(self, gps_raw):
		self.gps.measure_gps(gps_raw)

		self.gps.correct()

	def Talk( self ):

		msg = Odometry()
		msg.header.stamp = rospy.Time.now()

		msg.pose.pose.position.x = self.gps.position.x
		msg.pose.pose.position.y = self.gps.position.y
		msg.pose.pose.position.z = self.gps.position.z 

		self.publisher['state'].publish(msg) 

def main():

	rospy.init_node('GPS_Test', anonymous = True)
	node = ROS_GPS()

	rospy.spin()

if __name__ == "__main__": main()
