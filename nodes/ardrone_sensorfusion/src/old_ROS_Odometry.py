#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_sensorfusion')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry 
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

from Odometry import Odometry as AROdometry
from Quadrotor import Quadrotor 
from math import pi

import tf

# Some Constants
Command_Time = 0.005;
g = 9.81;

class SensorFusion(object):
    """docstring for SensorFusion"""
    def __init__(self, odometry = AROdometry(Ts = Command_Time) ):
        super(SensorFusion, self).__init__()
        rospy.init_node('SensorFusion_Odometry', anonymous = True)

        rospy.Subscriber('ardrone/navdata',Navdata, callback = self.Listen, callback_args = self.ReceiveNavdata) 
        rospy.Subscriber('ardrone/imu', Imu, callback = self.Listen, callback_args = self.ReceiveImu)
        rospy.Subscriber('ardrone/sonar_height', Imu, callback = self.Listen, callback_args = self.ReceiveSonarHeight)

        self.publisher = rospy.Publisher('ardrone/sensorfusion/navdata', Odometry)

        self.odometry = odometry


    def Listen(self, data, function):
        function(data)

    def Talk(self):
        msgs = Odometry()
        msgs.header.stamp = rospy.Time.now()
        msgs.header.frame_id = "/local"


        msgs.pose.pose.position.x = self.odometry.position.x
        msgs.pose.pose.position.y = self.odometry.position.y
        msgs.pose.pose.position.z = self.odometry.position.z

        quaternion = tf.transformations.quaternion_from_euler(self.odometry.position.yaw, self.odometry.position.pitch, self.odometry.position.roll, 'rzyx')
        
        msgs.pose.pose.orientation.x = quaternion[0]
        msgs.pose.pose.orientation.y = quaternion[1]
        msgs.pose.pose.orientation.z = quaternion[2]
        msgs.pose.pose.orientation.w = quaternion[3]
        
        msgs.twist.twist.linear.x = self.odometry.velocity.x
        msgs.twist.twist.linear.y = self.odometry.velocity.y
        msgs.twist.twist.linear.z = self.odometry.velocity.z

        #rospy.loginfo(msgs)
        
        self.publisher.publish(msgs)


    def ReceiveNavdata(self, navdata):
        self.odometry.position = dict(roll = navdata.rotX * 180.0 / pi, pitch = navdata.rotY * 180.0 / pi, yaw = navdata.rotZ * 180.0 / pi );
        self.odometry.velocity = dict(x = navdata.vx / 1000.0, y = navdata.vy / 1000.0, z = navdata.vz / 1000.0)
        self.odometry.acceleration = dict(x = navdata.ax * g, y = navdata.ay * g, z = navdata.az * g)

        self.odometry.state = navdata.state
        self.odometry.battery = navdata.batteryPercent

        self.odometry.predict()

        self.Talk()

    def ReceiveImu(self, ImuRaw):
        pass

    def ReceiveSonarHeight(self, Range):
        pass


def main():
    node = SensorFusion()
    rospy.spin()

    

if __name__ == "__main__": main()