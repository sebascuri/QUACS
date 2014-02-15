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



def test_gps():
	print "New Test"
	Ts = 1
	Fusion = SensorFusion( sensors = [Sensors.GPS(), Sensors.DummyYaw()], processes = [Process.XY_Odometry1( Ts = Ts ) , Process.Z_Odometry1( Ts = Ts)] )
	Fusion.velocity.set_properties( dict(x=2, y = 3, z = -1) )

	print Fusion.sensors
	for sensor in Fusion.sensors:
		print sensor, len(sensor)
		print sensor == 'GPS'
		print sensor.Z 

	print Fusion.Z 
	for process in Fusion.processes:
		print repr(process), len(process)
		print process.predict()
		print process.predict()

	print Fusion.X 
	print Fusion.ProcessJacobian
	Fusion.predict() 
	print Fusion.X 
	print Fusion.ProcessJacobian
	print Fusion.MeasurementJacobian
	Fusion.predict(); print 'X', Fusion.X 
	Fusion.correct(); print Fusion.X 
	print Fusion.MeasurementJacobian

	for sensor in Fusion.sensors:
		if sensor == 'GPS':
			sensor.measure(latitude = 0.1, altitude = 2, longitude = 12)
			print sensor.Z 

	print"Predict"; Fusion.predict(); print Fusion.X; print Fusion.position
	print"Correct";Fusion.correct(); print Fusion.X; print Fusion.position
	
	print vars(Fusion)

def test_imu():
	imu = IMU_Kalman( Ts = 0.01 )
	imu.predict()
	imu.correct()

	print vars(imu)
							
def main():
	test_imu()
	# print 'Z =', Fusion.Z
	# print 'X =', Fusion.X 


class SensorFusion( object):
	"""docstring for SensorFusion:
	Fuses a big list of processes and sensors, 

	Predicts using the processes, 
	Corrects using the sensors and a Kalman Filter 
	"""
	def __init__(self, **kwargs):
		super(SensorFusion, self).__init__(** kwargs)

		self.init_processes()
		self.init_sensors()
			
	def __len__(self):

		return len(self.stateList)

	def init_processes(self):
		for process in self.processes:
			self.stateList += process.stateList # makes global state list 
			if self.X is not None: # initializes X 
				self.X = np.bmat([ [self.X], [process.X] ])

				self.ProcessJacobian = sp.block_diag(self.ProcessJacobian, process.Jacobian)
				self.ProcessCovariance = sp.block_diag(self.ProcessCovariance, process.Covariance)
			else:
				self.X = process.X

				self.ProcessJacobian = process.Jacobian
				self.ProcessCovariance = process.Covariance

	def init_sensors(self):
		for sensor in self.sensors:
			self.measurementList += sensor.measurementList

		for sensor in self.sensors:
			if self.Z is not None:
				self.Z = np.bmat( [ [self.Z], [sensor.Z] ])
				self.StateMap = np.bmat( [ [ self.StateMap], [sensor.StateMap] ] )
				self.MeasurementCovariance = sp.block_diag(self.MeasurementCovariance, sensor.Covariance)
			else:
				self.Z = sensor.Z
				self.StateMap = sensor.StateMap

				self.MeasurementCovariance = sensor.Covariance

			self.MeasurementJacobian = np.matlib.zeros([ len(self.measurementList), len(self.stateList) ])
			for measured_variable in sensor.measurementList:
				for state in sensor.stateList:
					self.MeasurementJacobian[self.measurementList.index(measured_variable), self.stateList.index(state) ] = sensor.Jacobian[sensor.measurementList.index(measured_variable), sensor.stateList.index(state) ]

	def predict(self):
		""" Predicts state X with quadrotor physics 
		Predicts Error Covariance 
		"""
		super(SensorFusion, self).predict() # this generates the X vector of each process

		# This generates the Global X vector 
		i = 0
		for process in self.processes:
			self.X[i:i+len(process)] = process.X  

			self.ProcessJacobian[ i:i+len(process), i:i+len(process) ] = process.Jacobian

			i += len(process )



		self.ErrorCovariance = self.ProcessJacobian * self.ErrorCovariance * self.ProcessJacobian.transpose() + self.ProcessCovariance
		# P = J . P . J' + Q

		""" self.processes = kwargs.get('processes', self.processes)

		i = 0
		for process in self.processes:
			process_size = len(process.stateList)
			print process_size
			print self.X 
			print (i + 1 + process_size)
			self.X[ i:(i + 1 + process_size) ][0] = process.X[:]
			print self.X 
			self.ProcessJacobian[ i:(i + 1 + process_size) ][ i:(i + 1 + process_size) ] = process.Jacobian

			i += process_size

		

		# self.X = kwargs.get('X', self.X) # X = f(X)
		# self.ProcessJacobian = kwargs.get('Jacobian', self.ProcessJacobian)
		# self.ProcessCovariance = kwargs.get('Covariance', self.ProcessCovariance)

		
		# np.dot(np.dot(self.ProcessJacobian, self.ErrorCovariance), np.transpose(self.ProcessJacobian)) + self.ProcessCovariance
		# P = J . P . J' + Q

		# return self.X, self.stateList

		""" 

	def correct(self):
		"""
		Calculates Kalman Gain 
		Correct X Estimation
		Corrects Error Estitmation
		"""

		# This generates the Global Z and StateMap vectors , sensor Z and StateMap vectors are already generated
		i = 0
		for sensor in self.sensors:
			self.Z[i:i+len(sensor)] = sensor.Z 
			self.StateMap[i:i+len(sensor)] = sensor.StateMap
			print 'StateMap', sensor.StateMap, self.StateMap
			i += len(sensor)

			for measured_variable in sensor.measurementList:
				for state in sensor.stateList:
					self.MeasurementJacobian[self.measurementList.index(measured_variable), self.stateList.index(state) ] = sensor.Jacobian[sensor.measurementList.index(measured_variable), sensor.stateList.index(state) ]
			

		print 'Z', self.Z , 'StateMap', self.StateMap
		self.KalmanGain = self.ErrorCovariance * self.MeasurementJacobian.transpose() *  np.linalg.inv( self.MeasurementJacobian * self.ErrorCovariance * self.MeasurementJacobian.transpose() + self.MeasurementCovariance )
		# K = P . H' . ( H . P . H' + R ) ^ -1; 
		
		print "Size = ", np.shape(self.KalmanGain)
		print len(self)
		self.X += self.KalmanGain * ( self.Z - self.StateMap)
		# X = X + K . ( Z - h(X) )
		self.ErrorCovariance = (matlib.eye(np.size( self.X ) ) - self.KalmanGain * self.MeasurementJacobian ) * self.ErrorCovariance
		# P = ( I - K . H) . P

		self.set_attributes(self.X, self.stateList)


		"""
		i = 0
		for sensor in sensors:
			sensor_size = len(sensor.measurementList)
			self.Z[ i:(i + 1 + sensor_size) ][0] = sensor.Z 
			self.StateMap[ i:(i + 1 + sensor_size) ][0] = sensor.StateMap

			self.MeasurementJacobian[ i:(i + 1 + sensor_size) ][ i:(i + 1 + sensor_size) ] = sensor.Jacobian

			i += sensor_size


		# self.Z = kwargs.get('Z', self.Z)
		# self.MeasurementJacobian = kwargs.get('Jacobian', self.MeasurementJacobian)
		# self.MeasurementCovariance = kwargs.get('Covariance', self.MeasurementCovariance)
		# self.StateMap = kwargs.get('StateMap', self.StateMap)

		# K1 = np.dot( self.ErrorCovariance, np.transpose(self.MeasurementJacobian) )
		# K2 = np.dot( np.dot( self.MeasurementJacobian, self.ErrorCovariance ), np.transpose(self.MeasurementJacobian) ) + self.MeasurementCovariance
		# self.KalmanGain = np.dot( K1, np.linalg.inv(K2) )

		# K = P . H' . ( H . P . H' + R ) ^ -1; 
		# K1 = P . H'; K2 = H . P . H' + R

		# self.X += np.dot( self.KalmanGain, self.Z - self.map(self.X ) )
		# X = X + K . ( Z - h(X) )
		

		# self.ErrorCovariance = np.dot( np.identity( np.size(self.X) ) - np.dot( self.KalmanGain, self.MeasurementJacobian ), self.ErrorCovariance )
		# P = ( I - K . H) . P

		# return self.X, self.stateList
		"""
	
	# Object Properties
	@property
	def stateList(self):
		return self.properties.get('stateList', list())
	@stateList.setter
	def stateList(self, new_list):
		self.properties['stateList'] = new_list
	@stateList.deleter
	def stateList(self):
		del self.properties['stateList']

	@property
	def measurementList(self):
		return self.properties.get('measurementList', list())
	@measurementList.setter
	def measurementList(self, new_list):
		self.properties['measurementList'] = new_list
	@measurementList.deleter
	def measurementList(self):
		del self.properties['measurementList']

	@property
	def X(self):
		return self.properties.get('X', None)
	@X.setter
	def X(self, X):
		self.properties['X'] = X
	@X.deleter
	def X(self):
		del self.properties['X']

	@property
	def Z(self):
		return self.properties.get('Z', None)
	@Z.setter
	def Z(self, Z):
		self.properties['Z'] = Z
	@Z.deleter
	def Z(self):
		del self.properties['Z']

	@property
	def StateMap(self):
		return self.properties.get('StateMap', None)
	@StateMap.setter
	def StateMap(self, StateMap):
		self.properties['StateMap'] = StateMap
	@StateMap.deleter
	def StateMap(self):
		del self.properties['StateMap']

	@property 
	def ProcessJacobian(self):
		return self.properties.get('ProcessJacobian', None)
	@ProcessJacobian.setter
	def ProcessJacobian(self, Q):
		self.properties['ProcessJacobian'] = Q
	@ProcessJacobian.deleter
	def ProcessJacobian(self):
		del self.properties['ProcessJacobian']

	@property 
	def MeasurementJacobian(self):
		return self.properties.get('MeasurementJacobian', None)
	@MeasurementJacobian.setter
	def MeasurementJacobian(self, Q):
		self.properties['MeasurementJacobian'] = Q
	@MeasurementJacobian.deleter
	def MeasurementJacobian(self):
		del self.properties['MeasurementJacobian']

	@property 
	def ErrorCovariance(self):
		return self.properties.get('ErrorCovariance', np.matlib.zeros([len(self), len(self)]) )
	@ErrorCovariance.setter
	def ErrorCovariance(self, P):
		self.properties['ErrorCovariance'] = P
	@ErrorCovariance.deleter
	def ErrorCovariance(self):
		del self.properties['ErrorCovariance']

	@property 
	def ProcessCovariance(self):
		return self.properties.get('ProcessCovariance', None)
	@ProcessCovariance.setter
	def ProcessCovariance(self, Q):
		self.properties['ProcessCovariance'] = Q
	@ProcessCovariance.deleter
	def ProcessCovariance(self):
		del self.properties['ProcessCovariance']

	@property 
	def MeasurementCovariance(self):
		return self.properties.get('MeasurementCovariance', None)
	@MeasurementCovariance.setter
	def MeasurementCovariance(self, R):
		self.properties['MeasurementCovariance'] = R 
	@MeasurementCovariance.deleter
	def MeasurementCovariance(self):
		del self.properties['MeasurementCovariance']

	@property
	def KalmanGain(self):
		return self.properties.get('KalmanGain', np.matlib.zeros([len(self), len(self)]))
	@KalmanGain.setter
	def KalmanGain(self, K):
		self.properties['KalmanGain'] = K 
	@KalmanGain.deleter
	def KalmanGain(self):
		del self.properties['KalmanGain']
	
if __name__ == '__main__': main()



