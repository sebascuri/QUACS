#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi, cos, sin
import numpy as np 
import numpy.matlib as matlib
import scipy.linalg as sp 

# from Odometry import Odometry1 as Odometry 
import Process
import Sensors
from Quadrotor import Quadrotor 
# from BasicObject import BasicObject

class SensorFusion(Quadrotor, object):
	"""docstring for SensorFusion"""
	# def __init__(self, ErrorCovariance = np.zeros([6,6]), ProcessCovariance = 0.0003 * np.identity(6), MeasurementCovariance = 0.6  * np.identity(6), **kwargs):
	def __init__(self, **kwargs):
		super(SensorFusion, self).__init__(** kwargs)

		self._stateList = list()
		self._measurementList = list()


		for process in self.processes:
			self._stateList += process._stateList # makes global state list 
			if hasattr(self, 'X'): # initializes X 
				self.X = np.bmat([ [self.X], [process.X] ])

				self.ProcessJacobian = sp.block_diag(self.ProcessJacobian, process.Jacobian)
				self.ProcessCovariance = sp.block_diag(self.ProcessCovariance, process.Covariance)
			else:
				self.X = process.X

				self.ProcessJacobian = process.Jacobian
				self.ProcessCovariance = process.Covariance
			

		for sensor in self.sensors:
			self._measurementList += sensor._measurementList

		self.MeasurementJacobian = np.matlib.zeros([ len(self._measurementList), len(self._stateList) ])

		for sensor in self.sensors:
			if hasattr(self, 'Z'):
				self.Z = np.bmat( [ [self.Z], [sensor.Z] ])
				self.Residual = np.bmat( [ [ self.Residual], [sensor.Residual] ] )
				self.MeasurementCovariance = sp.block_diag(self.MeasurementCovariance, sensor.Covariance)
			else:
				self.Z = sensor.Z
				self.Residual = sensor.Residual

				self.MeasurementCovariance = sensor.Covariance

			for measured_variable in sensor._measurementList:
				for state in sensor._stateList:
					self.MeasurementJacobian[self._measurementList.index(measured_variable), self._stateList.index(state) ] = sensor.Jacobian[sensor._measurementList.index(measured_variable), sensor._stateList.index(state) ]
			
					# self.MeasurementJacobian[ self._measurementList.index(measured_variable) ][ self._stateList.index(state) ] = sensor.Jacobian[ sensor._measurementList.index(measured_variable) ][ sensor._stateList.index(state) ]
			

		self.ErrorCovariance = np.matlib.zeros([len(self._stateList), len(self._stateList)])


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
			process_size = len(process._stateList)
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

		# return self.X, self._stateList

		""" 

	def correct(self):
		"""
		Calculates Kalman Gain 
		Correct X Estimation
		Corrects Error Estitmation
		"""

		# This generates the Global Z and Residual vectors , sensor Z and Residual vectors are already generated
		i = 0
		for sensor in self.sensors:
			self.Z[i:i+len(sensor)] = sensor.Z 
			self.Residual[i:i+len(sensor)] = sensor.Residual
			print 'Residual', sensor.Residual, self.Residual
			i += len(sensor)

			for measured_variable in sensor._measurementList:
				for state in sensor._stateList:
					self.MeasurementJacobian[self._measurementList.index(measured_variable), self._stateList.index(state) ] = sensor.Jacobian[sensor._measurementList.index(measured_variable), sensor._stateList.index(state) ]
			

		print 'Z', self.Z , 'Residual', self.Residual
		self.KalmanGain = self.ErrorCovariance * self.MeasurementJacobian.transpose() *  np.linalg.inv( self.MeasurementJacobian * self.ErrorCovariance * self.MeasurementJacobian.transpose() + self.MeasurementCovariance )
		# K = P . H' . ( H . P . H' + R ) ^ -1; 
		
		self.X += self.KalmanGain * ( self.Z - self.Residual)
		# X = X + K . ( Z - h(X) )
		self.ErrorCovariance = (matlib.eye(np.size( self.X ) ) - self.KalmanGain * self.MeasurementJacobian ) * self.ErrorCovariance
		# P = ( I - K . H) . P

		self.set_attributes(self.X, self._stateList)


		"""
		i = 0
		for sensor in sensors:
			sensor_size = len(sensor._measurementList)
			self.Z[ i:(i + 1 + sensor_size) ][0] = sensor.Z 
			self.Residual[ i:(i + 1 + sensor_size) ][0] = sensor.Residual

			self.MeasurementJacobian[ i:(i + 1 + sensor_size) ][ i:(i + 1 + sensor_size) ] = sensor.Jacobian

			i += sensor_size


		# self.Z = kwargs.get('Z', self.Z)
		# self.MeasurementJacobian = kwargs.get('Jacobian', self.MeasurementJacobian)
		# self.MeasurementCovariance = kwargs.get('Covariance', self.MeasurementCovariance)
		# self.Residual = kwargs.get('Residual', self.Residual)

		# K1 = np.dot( self.ErrorCovariance, np.transpose(self.MeasurementJacobian) )
		# K2 = np.dot( np.dot( self.MeasurementJacobian, self.ErrorCovariance ), np.transpose(self.MeasurementJacobian) ) + self.MeasurementCovariance
		# self.KalmanGain = np.dot( K1, np.linalg.inv(K2) )

		# K = P . H' . ( H . P . H' + R ) ^ -1; 
		# K1 = P . H'; K2 = H . P . H' + R

		# self.X += np.dot( self.KalmanGain, self.Z - self.map(self.X ) )
		# X = X + K . ( Z - h(X) )
		

		# self.ErrorCovariance = np.dot( np.identity( np.size(self.X) ) - np.dot( self.KalmanGain, self.MeasurementJacobian ), self.ErrorCovariance )
		# P = ( I - K . H) . P

		# return self.X, self._stateList
		"""
	"""
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
	def Residual(self):
		return self.properties.get('Residual', None)
	@Residual.setter
	def Residual(self, Residual):
		self.properties['Residual'] = Residual
	@Residual.deleter
	def Residual(self):
		del self.properties['Residual']

	@property 
	def ErrorCovariance(self):
		return self.properties.get('ErrorCovariance', None)
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
		return self.properties.get('KalmanGain', None)
	@KalmanGain.setter
	def KalmanGain(self, K):
		self.properties['KalmanGain'] = K 
	@KalmanGain.deleter
	def KalmanGain(self):
		del self.properties['KalmanGain']

	@property
	def sensors(self):
		return self.properties.get('sensors', None)
	@sensors.setter
	def sensors(self, sensors):
		self.properties['sensors'] = sensors
	@sensors.deleter
	def sensors(self):
		del self.properties['sensors']

	@property
	def processes(self):
		return self.properties.get('processes', None)
	@processes.setter
	def processes(self, processes):
		self.properties['processes'] = processes
	@processes.deleter
	def processes(self):
		del self.properties['processes']
	"""

					
def main():
	print "New Test"
	Ts = 1
	Fusion = SensorFusion( sensors = [Sensors.GPS(), Sensors.DummyYaw()], processes = [Process.XY_Odometry1( Ts = Ts ) , Process.Z_Odometry1( Ts = Ts)] )
	Fusion.velocity.set_attribute( dict(x=2, y = 3, z = -1) )

	print Fusion.sensors
	for sensor in Fusion.sensors:
		print sensor, len(sensor)
		print sensor == 'GPS'
		print sensor.Z 

	print Fusion.Z 
	for process in Fusion.processes:
		print repr(process), len(process)
		print process.update()
		print process.update()

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


	# print 'Z =', Fusion.Z
	# print 'X =', Fusion.X 
	
if __name__ == '__main__': main()
