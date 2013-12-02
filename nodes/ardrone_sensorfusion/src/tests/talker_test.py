#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_sensorfusion')
import rospy

from ardrone_sensorfusion.msg import Navdata as myNavdata
from nav_msgs.msg import Odometry

from ardrone_autonomy.msg import Navdata as arNavdata # for receiving navdata feedback
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

def talker():
	#pub = rospy.Publisher('/ardrone_sensorfusion', myNavdata)
	pub = rospy.Publisher('ardrone/sensorfusion/navdata', Odometry)
	print "ok"
	rospy.init_node('SensorFusion_Test', anonymous = True)

	print "start"

	while not rospy.is_shutdown():
		msgs = Odometry()
		msgs.header.stamp = rospy.Time.now()
		msgs.header.frame_id = "/ned"
		
		#rospy.loginfo(msgs)

		rospy.sleep(1.0)

		pub.publish(msgs)

    


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
