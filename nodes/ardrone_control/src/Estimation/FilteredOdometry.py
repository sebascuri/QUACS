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
from Filter import DigitalFilter
from ROS_Odometry import ROS_Odometry 


from math import pi

import tf

# Some Constants
Command_Time = 0.005;
g = 9.81;

class FilteredOdometry(ROS_Odometry, object):
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

    low_pass_filters dictionary from navdata entries to corresponding butterworth filter 

    """
    def __init__(self, **kwargs ):
        super(FilteredOdometry, self).__init__(**kwargs)
        
        self.low_pass_filters = dict( 
            vz = DigitalFilter( a = [-2.369513007182038, 2.313988414415879, -1.054665405878567, 0.187379492368185], b = [0.004824343357716, 0.019297373430865, 0.028946060146297, 0.019297373430865, 0.004824343357716]),
            rotZ = DigitalFilter( a = [-2.369513007182038, 2.313988414415879, -1.054665405878567, 0.187379492368185], b = [0.004824343357716, 0.019297373430865, 0.028946060146297, 0.019297373430865, 0.004824343357716]),
            ax = DigitalFilter( a = [-1.955578240315035, 0.956543676511203], b = [0.000241359049041961, 0.000482718098083923, 0.000241359049041961]),
            ay = DigitalFilter( a = [-1.955578240315035, 0.956543676511203], b = [0.000241359049041961, 0.000482718098083923, 0.000241359049041961]),
            az =  DigitalFilter( a = [-3.794791103079410, 5.405166861726178, -3.424747347274252, 0.814405997727281], b = [0.02150568737306, 0.086022749492265, 0.129034124238397, 0.086022749492265, 0.02150568737306]),
            )

    @property 
    def low_pass_filters(self):
        return self.properties.get('low_pass_filters', None)
    @low_pass_filters.setter 
    def low_pass_filters(self, low_pass_filters):
        self.properties['low_pass_filters'] = low_pass_filters
    @low_pass_filters.deleter
    def low_pass_filters(self):
        del self.properties['low_pass_filters']
        
    def ReceiveNavdata(self, navdata):
        filtered_navdata = self.FilterNavdata(navdata)
        super(FilteredOdometry, self).ReceiveNavdata(filtered_navdata)

    def FilterNavdata(self, navdata):
        for key in self.low_pass_filters.keys():
            self.low_pass_filters[key].set_input( getattr(navdata, key) ) #input new data in filter
            setattr(navdata, key, self.low_pass_filters[key].get_output() ) #set in navdata filter's output

        return navdata

def main():
    rospy.init_node('SensorFusion_Odometry', anonymous = True)
    node = FilteredOdometry( processes = [Process.XY_Odometry1(Ts = Command_Time), Process.Z_Odometry1(Ts = Command_Time) ] )
    rospy.spin()

    

if __name__ == "__main__": main()