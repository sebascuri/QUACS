#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range, NavSatFix
from nav_msgs.msg import Odometry 
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# from Odometry import Odometry as AROdometry
from Quadrotor import Quadrotor 
import Process

from math import pi

import tf

# Some Constants
Command_Time = 0.005;
g = 9.81;

class SensorFusion(Quadrotor, object):
    """docstring for SensorFusion"""
    def __init__(self, **kwargs ):
        super(SensorFusion, self).__init__(**kwargs)
        rospy.Subscriber('/ardrone/navdata',Navdata, callback = self.ReceiveNavdata ) 
        rospy.Subscriber('/fix', NavSatFix, callback = self.ReceiveGPS)
        rospy.Subscriber('/ardrone/imu', Imu, callback = self.ReceiveImu )
        rospy.Subscriber('/ardrone/sonar_height', Imu, callback = self.ReceiveSonarHeight )

        self.name = kwargs.get('name', "/local")


        self.publisher = rospy.Publisher('/ardrone/sensorfusion/navdata', Odometry)

        self.local_tf = tf.TransformBroadcaster()

        rospy.Timer(rospy.Duration( Command_Time ), self.Command, oneshot=False)

    def Command(self, time):
        self.Talk()
        self.predict()


    def Talk(self):
        msgs = Odometry()
        msgs.header.stamp = rospy.Time.now()
        msgs.header.frame_id = "/nav"
        msgs.child_frame_id = self.name 


        msgs.pose.pose.position.x = self.position.x
        msgs.pose.pose.position.y = self.position.y
        msgs.pose.pose.position.z = self.position.z

        # quaternion = tf.transformations.quaternion_from_euler(self.position.yaw, self.position.pitch, self.position.roll, 'rzyx')
        
        msgs.pose.pose.orientation.x = self.orientation.x
        msgs.pose.pose.orientation.y = self.orientation.y
        msgs.pose.pose.orientation.z = self.orientation.z
        msgs.pose.pose.orientation.w = self.orientation.w 
        
        msgs.twist.twist.linear.x = self.velocity.x
        msgs.twist.twist.linear.y = self.velocity.y
        msgs.twist.twist.linear.z = self.velocity.z

        #rospy.loginfo(msgs)
        
        self.publisher.publish(msgs)

        self.local_tf.sendTransform( (msgs.pose.pose.position.x, msgs.pose.pose.position.y, msgs.pose.pose.position.z) , 
            (msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y, msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w), 
            msgs.header.stamp, 
            msgs.child_frame_id, 
            msgs.header.frame_id)


    def ReceiveNavdata(self, navdata):
        self.position.set_attribute(dict(roll = navdata.rotX * pi/180.0, pitch = navdata.rotY * pi/180.0, yaw = navdata.rotZ * pi/180.0))
        self.velocity.set_attribute(dict(x = navdata.vx / 1000.0, y = navdata.vy / 1000.0, z = navdata.vz / 1000.0))
        self.acceleration.set_attribute(dict(x = navdata.ax * g, y = navdata.ay * g, z = (navdata.az - 1.0) * g ))
        self.orientation.set_euler(roll = navdata.rotX * pi/180.0 , pitch = navdata.rotY * pi/180.0 , yaw = navdata.rotZ * pi/180.0  )
        
        self.set_state(navdata.state)
        self.battery = navdata.batteryPercent

        
    def ReceiveGPS(self, gpsdata):
        pass

    def ReceiveImu(self, ImuRaw):
        """ Receive RAW imu msgs""" 
        pass

    def ReceiveSonarHeight(self, Range):
        """ Receive Sonar Heigh proximity msgs"""
        pass


def main():
    rospy.init_node('SensorFusion_Odometry', anonymous = True)
    node = SensorFusion( processes = [Process.XY_Odometry1(Ts = Command_Time), Process.Z_Odometry1(Ts = Command_Time) ] )
    rospy.spin()

    

if __name__ == "__main__": main()