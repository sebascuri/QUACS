#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range, NavSatFix
from nav_msgs.msg import Odometry 
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from diagnostic_msgs.msg import KeyValue # added for State Handling

# from Odometry import Odometry as AROdometry
from Quadrotor import Quadrotor 
import Process
import SensorFusion
from ROS import ROS_Object 

from math import pi

import tf

# Some Constants
Command_Time = 0.005;
IMU_Period = 0.01
g = 9.81;

class ROS_SensorFusion(Quadrotor, ROS_Object, object):
    """docstring for SensorFusion
    Subscribed to:
        ardrone/navdata -> reads ardrone raw navdata 
        fix -> reads gps 
        ardrone/imu -> reads imu for complementary filter 
        ardrone/sonar_height -> for Z proximity 

    Publishes to:
        /ardrone/sensorfusion/navdata -> Odometry estimated state of ardrone 
            It is called by a Timer
            Published as a tf too. 

    """
    
    def __init__(self, **kwargs ):
        super(ROS_SensorFusion, self).__init__(**kwargs)
        self.name = kwargs.get('name', "/local")

        self.subscriber.update(
            raw_navdata = rospy.Subscriber('/ardrone/navdata',Navdata, callback = self.ReceiveNavdata ),
            raw_gps = rospy.Subscriber('/fix', NavSatFix, callback = self.ReceiveGPS),
            raw_imu = rospy.Subscriber('/ardrone/imu', Imu, callback = self.ReceiveImu ),
            # raw_sonar = rospy.Subscriber('/sonar_height', Range, callback = self.ReceiveSonarHeight )
            )

        self.publisher.update(
            state = rospy.Publisher('/ardrone/sensorfusion/navdata', Odometry)
            )

        self.tf_broadcaster.update( 
            drone_local_tf = tf.TransformBroadcaster( )
            )

        self.timer.update( 
            publish_timer = rospy.Timer(rospy.Duration( IMU_Period ), self.Talk, oneshot=False)
            )

    def Talk(self, time):
        #print self.orientation.get_euler()['yaw'], self.position.yaw, self.sensors['gps'].position.yaw
        msgs = Odometry( )
        msgs.header.stamp = rospy.Time.now()
        msgs.header.frame_id = "/nav"

        msgs.child_frame_id = "/drone_local"

        msgs.pose.pose.position.x = self.position.x
        msgs.pose.pose.position.y = self.position.y
        msgs.pose.pose.position.z = self.position.z
        
        msgs.pose.pose.orientation.x = self.orientation.x
        msgs.pose.pose.orientation.y = self.orientation.y
        msgs.pose.pose.orientation.z = self.orientation.z
        msgs.pose.pose.orientation.w = self.orientation.w
        
        msgs.twist.twist.linear.x = self.velocity.x
        msgs.twist.twist.linear.y = self.velocity.y
        msgs.twist.twist.linear.z = self.velocity.z

        #rospy.loginfo(msgs)
        
        self.publisher['state'].publish(msgs)

        self.tf_broadcaster['drone_local_tf'].sendTransform( (msgs.pose.pose.position.x, msgs.pose.pose.position.y, msgs.pose.pose.position.z) , 
            self.orientation.get_quaternion( ), 
            msgs.header.stamp,
            msgs.child_frame_id, 
            msgs.header.frame_id)

    def ReceiveNavdata(self, navdata):
        try:
            self.sensors['gps'].measure_navdata(navdata) 
            self.sensors['gps'].predict()
            
        except KeyError:
            self.position.set_properties(dict(roll = navdata.rotX * pi/180.0, pitch = navdata.rotY * pi/180.0, yaw = navdata.rotZ * pi/180.0))
            self.velocity.set_properties(dict(x = navdata.vx / 1000.0, y = navdata.vy / 1000.0, z = navdata.vz / 1000.0))
            self.predict()

        self.acceleration.set_properties(dict(x = navdata.ax * g, y = navdata.ay * g, z = (navdata.az - 1.0) * g ))
        
        self.set_state(navdata.state)
        self.battery = navdata.batteryPercent

    def ReceiveGPS(self, gpsdata):
        try: 
            self.sensors['gps'].measure_gps(gpsdata)
            self.sensors['gps'].correct()
        except KeyError:
            print "No GPS"

    def ReceiveImu(self, imu_raw):
        try:
            self.sensors['imu'].measure(imu_raw)
            self.sensors['imu'].predict()
            self.sensors['imu'].correct()

            # self.orientation = self.imu.quaternion
            try:
                #pass
                self.sensors['gps'].position.set_properties( self.orientation.get_euler( ) ) #set euler angles from filter
            except KeyError:
                #pass
                self.position.set_properties( self.orientation.get_euler( ) ) #set euler angles from filter

        except KeyError:
            print "No IMU"

    def ReceiveSonarHeight(self, Range):
        """ Receive Sonar Heigh proximity msgs"""
        pass


def main():
    rospy.init_node('SensorFusion_Odometry', anonymous = True)
    node = ROS_SensorFusion( 
        sensors = dict( imu = SensorFusion.IMU_Magdwick(Ts = IMU_Period) ), 
        position = dict( x = 2.0, y = -3.0, z = 1.0, yaw = 0.0, pitch = 0.0, roll = 0.0)
        )
    # gps = SensorFusion.GPS_Filter(Ts = Command_Time) ) 
    
    rospy.spin()

    

if __name__ == "__main__": main()