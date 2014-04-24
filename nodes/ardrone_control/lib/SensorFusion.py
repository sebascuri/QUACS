#!/usr/bin/env python

from math import pi, cos, sin, sqrt
import numpy as np 
import numpy.matlib as matlib
import numpy.linalg as linalg
import scipy.linalg as sp 


try:
	import tf;
except ImportError:
	import roslib; roslib.load_manifest('ardrone_control')
	import rospy; 
	import tf;


# from Odometry import Odometry1 as Odometry 
import Process
import Sensors
import Filter
# from Quadrotor import Quadrotor 
from BasicObject import BasicObject, SixDofObject, Quaternion

# from BasicObject import BasicObject

R = 6378137.0
class IMU_Filter(Sensors.IMU, Process.SO3, object):
	def __init__(self, **kwargs):
		super(IMU_Filter, self).__init__(**kwargs)

	def __len__(self):
		return len(self.stateList)

	def measure(self, imu_raw):
		super(IMU_Filter, self).measure(imu_raw) #measure sensors 

		for attribute in self.sensors["gyroscope"].measurementList:
			setattr(self.velocity, attribute, getattr(self.sensors["gyroscope"], attribute)) 
			# set sensor measurements into process
	
class IMU_Kalman(IMU_Filter, Filter.ExtendedKalmanFilter, object):
	"""docstring for IMU_Kalman"""

	def __init__(self, **kwargs):
		super(IMU_Kalman, self).__init__(**kwargs)
		# self.Ts = kwargs.get('Ts', 0.010)
		# self.quaternion = kwargs.get('quaternion', Quaternion())

		self.Z = np.mat([0.0, 0.0, 0.0]).transpose()
		self.StateMap = np.mat([0.0, 0.0, 0.0]).transpose()

		

		self.ProcessCovariance  = np.mat( [
			[ 0.005, 0., 0., 0.],
			[ 0., 0.005, 0., 0.],
			[ 0., 0., 0.005, 0.],
			[ 0., 0., 0., 0.005],
			])

		self.MeasurementCovariance = 10*np.mat( [
			[1., 0., 0.],
			[0., 1., 0.],
			[0., 0., 1.]
			])

		self.ProcessJacobian = matlib.zeros( [len(self.stateList), len(self.stateList)] )
		self.MeasurementJacobian = matlib.zeros( [len(self.measurementList), len(self.stateList) ])

	def __len__(self):
		return len(self.stateList)

	def predict(self):
		super(IMU_Kalman, self).predict()
		self.predict_error()

	def correct( self ):	
		self.Z = np.mat([ 
			self.sensors['accelerometer'].roll, 
			self.sensors['accelerometer'].pitch,
			self.sensors['dummy_yaw'].yaw 
			]).transpose()

		self.StateMap = np.mat( tf.transformations.euler_from_quaternion( self.quaternion.get_quaternion(), axes = 'sxyz'  ) ).transpose();

		c11 = 1 - 2 * (self.quaternion.y**2 + self.quaternion.z**2)
		c33 = 1 - 2 * (self.quaternion.x**2 + self.quaternion.y**2)
		c21 = 2 * ( self.quaternion.x*self.quaternion.y + self.quaternion.w*self.quaternion.z )
		c31 = 2 * ( self.quaternion.x*self.quaternion.z - self.quaternion.w*self.quaternion.y )
		c32 = 2 * ( self.quaternion.y*self.quaternion.z + self.quaternion.w*self.quaternion.x )


		self.MeasurementJacobian = np.asmatrix( 
			matlib.diag( [2 * c33/sqrt(c33 ** 2 + c32 ** 2), 2 / sqrt(1 - c31**2), 2 * c11/sqrt(c11 ** 2 + c21 ** 2)] ) 
		) * np.mat([
			[ self.quaternion.x , self.quaternion.w, self.quaternion.z, self.quaternion.y],
			[ self.quaternion.y, -self.quaternion.z, self.quaternion.w, -self.quaternion.x ],
			[ self.quaternion.z, self.quaternion.y, self.quaternion.x, self.quaternion.w]
		 ]) + np.asmatrix(
		 	matlib.diag( [4 * c32/sqrt(c33 ** 2 + c32 ** 2), 0 , 4 * c21/sqrt(c11 ** 2 + c21 ** 2) ]) 
		 ) *  np.mat([
		 	[ 0, self.quaternion.x, self.quaternion.y, 0],
		 	[ 0, 0, 0, 0],
		 	[ 0, 0 , self.quaternion.y, self.quaternion.z]
		]) 

		super(IMU_Kalman, self).correct()
		self.set_quaternion()

class IMU_Magdwick(IMU_Filter, Filter.MagdwickFilter, object):
	def __init__(self, **kwargs):
		super(IMU_Magdwick, self).__init__(**kwargs)

class IMU_Mahoney(IMU_Filter, Filter.MahoneyFilter, object):
	def __init__(self, **kwargs):
		super(IMU_Mahoney, self).__init__(**kwargs)

class GPS_Filter(Process.Odometry, Filter.ExtendedKalmanFilter, object):
	def __init__(self, **kwargs):
		super(GPS_Filter, self).__init__(**kwargs)
		self.sensors = dict( gps = Sensors.GPS(), velocity = Sensors.Velocity() )
		self.velocity = self.sensors['velocity'].velocity 

		self.ProcessCovariance  = 1 * np.mat( [
			[ 1., 0., 0., 0.],
			[ 0., 1., 0., 0.],
			[ 0., 0., 1., 0.],
			[ 0., 0., 0., 1.],
			])

		self.MeasurementCovariance  = 0.01 * R**2 * np.mat( [
			[ 1., 0., 0., 0.],
			[ 0., 1., 0., 0.],
			[ 0., 0., 1., 0.],
			[ 0., 0., 0., 1.],
			])

		self.MeasurementJacobian = matlib.eye(len(self))

	def __len__(self):
		return len(self.stateList)

	def measure_gps(self, fix_data):
		if not self.sensors['gps'].calibrated:
			self.sensors['gps'].set_zero(fix_data)
			self.sensors['gps'].calibrated = True 

		self.sensors['gps'].measure(fix_data)

	def measure_navdata(self, navdata):
		self.sensors['velocity'].measure(navdata)

	def predict(self):
		super(GPS_Filter, self).predict()
		self.predict_error()

	def correct(self):
		#generate Z 
		self.Z = np.vstack( [self.sensors['gps'].Z, np.mat(self.position.yaw) ] )
		self.StateMap = np.mat([ self.position.x, self.position.y, self.position.z, self.position.yaw] ).transpose()

		super(GPS_Filter, self).correct()
		self.set_position()


def test_gps():
	print "New Test"
	gps = GPS_Filter( Ts = 0.001, position = dict(x = 1.0) )

def test_imu():
	imu = IMU_Kalman( Ts = 0.01 )
	imu.predict()
	imu.correct()

	print vars(imu)
							
def main():
	#test_imu()
	test_gps( )
	# print 'Z =', Fusion.Z
	# print 'X =', Fusion.X 

	
if __name__ == '__main__': main()



