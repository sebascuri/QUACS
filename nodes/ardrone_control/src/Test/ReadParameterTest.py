#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

def main():    
	rospy.init_node('Controller', anonymous = True)
	y = rospy.get_param('Gains')
	print y, type(y)

	print y['Y']['P']
	x = rospy.get_param('Gains/X')
	print x, type(x)

	rospy.spin()


if __name__ == "__main__": main()