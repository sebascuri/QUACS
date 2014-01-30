#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi, cos, sin, sqrt
import numpy as np 
import numpy.matlib as matlib
import gps_transforms

from BasicObject import BasicObject, SixDofObject, Quaternion

class Sensor(BasicObject, object):
	"""docstring for Sensor
		There are two sets of data, measured variables and state variables
		Transformation between state variables and measured variables can be the identity or something more complex. 


		To go from measured variables to state variables use measure method
			Measure method uses object variables and changes state of object. New measurements are passed by argument 
			It returns None.  


		To go from state variables to measured variables use map method
			Map method uses argument variables and does not change state of object 
			It returns the mapping in measurement space. 


		There is a list that maps Z (measurement vector) and measurement properties named _measurementList

	"""
	def __init__(self, **kwargs):
		# super(Sensor, self).__init__( **kwargs )

		self.Z = kwargs.get('Z', matlib.zeros( [len(self._measurementList), 1] ))
		self.Residual = kwargs.get('Residual', matlib.zeros( [len(self._measurementList), 1] ))
		self.Jacobian = kwargs.get('Jacobian', matlib.eye( len(self._measurementList) ) )
		self.Covariance = kwargs.get('Covariance', 0.3 * matlib.eye( len(self) ))

		try:
			self.position = kwargs.get('position', self.position)
		except AttributeError:
			self.position = SixDofObject()


		self.velocity = kwargs.get('velocity', SixDofObject())
		self.orientation = kwargs.get('orientation', Quaternion())
		self.acceleration = kwargs.get('acceleration', SixDofObject())
		self.Ts = kwargs.get('Ts', None)

		self.set_Z()
		
	def __len__(self):

		return np.size(self._measurementList) 

	def __eq__(self, data):

		return str(self) == data

	def measure(self, **kwargs):
		""" input sensor values and assing them to state Vector"""
		for key, value in kwargs.items():
			setattr(self, key, value)

		self.set_Z()

	def map(self, **kwargs):
		""" map state variables from Quadrotor and returns a vector of what it should have been measured """
		for key, value in kwargs.items():
			setattr(self, key, value)
			

	def get_measurementList(self):

		return self._measurementList

	def set_Z(self):
		i = 0
		for key in self._measurementList:
			self.Z[i] = getattr(self, key)
			i += 1

	"""
	@property 
	def Z(self):
		return self.properties.get('Z', None)
	@Z.setter 
	def Z(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( matlib.zeros( 1 ) ):
				self.properties['Z'] = arg
			elif type(arg) == type( dict() ):
				for key, value in arg.items():
					self.Z[ self._measurementList.index(key) ][0] = value
		
		for key, value in kwargs.items():
			self.Z[ self._measurementList.index(key) ][0] = value

	@Z.deleter
	def Z(self):
		del self.properties['Z']

	@property 
	def Residual(self):
		return self.properties.get('Residual', None)
	@Residual.setter 
	def Residual(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( matlib.zeros( 1 ) ):
				self.properties['Residual'] = arg
			elif type(arg) == type( dict() ):
				for key, value in arg.items():
					self.Residual[ self._measurementList.index(key) ][0] = value
		
		for key, value in kwargs.items():
			self.Residual[ self._measurementList.index(key) ][0] = value

	@Z.deleter
	def Z(self):
		del self.properties['Z']

	@property
	def Jacobian(self):
		return self.properties.get('Jacobian')
	@Jacobian.setter
	def Jacobian(self, H):
		self.properties['Jacobian'] = H
	@Jacobian.deleter
	def Jacobian(self):
		del self.properties['Jacobian']	

	@property
	def Covariance(self):
		return self.properties.get('Covariance', None)
	@Covariance.setter
	def Covariance(self, R):
		self.properties['Covariance'] = R
	@Covariance.deleter
	def Covariance(self):
		del self.properties['Covariance']	
	"""

class DummyYaw(Sensor, object):
	"""docstring for GPS"""
	def __init__(self, **kwargs):
		self._measurementList = ['yaw']
		self._stateList = ['position.yaw']
		
		self.Covariance = 0.1 * matlib.eye( len(self._measurementList) )

		self.yaw = kwargs.get('yaw', 0)
		super(DummyYaw, self).__init__(**kwargs)

	def map(self, **kwargs):
		super(DummyYaw, self).map(**kwargs)
		self.Residual[0] = kwargs.get('yaw', self.position.yaw)
	def measure(self, **kwargs):
		super(DummyYaw, self).measure(**kwargs)
		self.Z[0] = kwargs.get('yaw', self.position.yaw)
		self.Jacobian = matlib.eye( len(self._measurementList) )

class GPS_2(Sensor, object):
	"""docstring for GPS"""
	def __init__(self, latitude = 0, longitude = 0, altitude = 0, **kwargs):
		measurementList = ['latitude', 'longitude', 'altitude']
		stateList = ['position.x', 'position.y', 'position.z']
		

		super(GPS, self).__init__(measurementList = measurementList, stateList = stateList, **kwargs)
		self.Covariance= 0.3 * matlib.eye( len(self._measurementList) )

		self.gps_zero = dict(latitude = latitude, longitude = longitude, altitude = altitude) 

		self.latitude = latitude
		self.altitude = altitude
		self.longitude = longitude
		self.Z = dict(latitude = latitude, longitude = longitude, altitude = altitude)
		# self.measure()

	def set_zero(self, *args, **kwargs):
		if (len(args) is 0) and (len(kwargs) is 0):
			i = 0;
			for key in self._measurementList:
				self.gps_zero[ key ] = self.Z.item(i)
				i += 1
		else:
			if ( len(args) is 1 ) and ( type(args[0]) == type(dict()) ) :
				set_zero( **args[0] )
			elif len(args) == 3:
				i = 0
				for key in self._measurementList:
					self.gps_zero[ key ] = args[i] 
					i += 1
			else:
				for key, value in kwargs.items():
					self.gps_zero[ key ] = value

	def map(self, **kwargs):
		super(GPS, self).map(**kwargs)

		gps_coord = gps_transforms.cart2gps(x = kwargs.get('x', self.position.x), y = kwargs.get('y', self.position.y), z = kwargs.get('z', self.position.z) ) #+ gps_transforms.R 
		
		# self.Residual = self.Z 
		for key, value in gps_coord.items():
			self.Residual[self._measurementList.index(key)][0] = value + self.gps_zero[key]


	def measure(self, **kwargs):
		super(GPS, self).measure(**kwargs)

		"""self.Jacobian = np.linalg.inv( np.matrix( [ 
			[ - r * sin(lat) * cos(lon), - r * cos(lat) * sin(lon), R * cos(lat) * cos(lon) ] ,
			[ - r * sin(lat) * sin(lon),  r * cos(lat) * cos(lon), R * cos(lat) * sin(lon) ] , 
			[ r * cos(lat), 0 , R*sin(lat) ]  ] ) )		
		"""
		R = gps_transforms.R
		self.Jacobian = np.matrix( [
			[-(cos(self.longitude)*sin(self.latitude))/(R + self.altitude) , -(sin(self.latitude)*sin(self.longitude))/(R + self.altitude), cos(self.latitude)/(R + self.altitude) ], 
			[ -sin(self.longitude)/(cos(self.latitude)*(R + self.altitude)) , cos(self.longitude)/(cos(self.latitude)*(R + self.altitude)), 0], 
			[ (cos(self.latitude)*cos(self.longitude))/R , (cos(self.latitude)*sin(self.longitude))/R ,  sin(self.latitude)/R] ] )

	"""def set_initial_state(self, **kwargs):
		for key, value in kwargs.items():
			self.gps_zero[key] = value

		for arg in args:
			if type(arg) == type(dict()):
				for key, value in args.items():
					self.gps_zero[key] = value
	"""

class GPS(Sensor, object):
	"""docstring for GPS"""
	def __init__(self, **kwargs):
		self._measurementList = ['x', 'y', 'z']
		self._stateList = ['position.x', 'position.y', 'position.z']
		
		self.Covariance= 0.3 * matlib.eye( len(self._measurementList) )

		self.gps_zero = gps_transforms.gps2cart( latitude = kwargs.get('latitude', 0) , altitude = kwargs.get('altitude', 0) , longitude = kwargs.get('longitude', 0) )

		self.x = kwargs.get('x', 0)
		self.y = kwargs.get('y', 0)
		self.z = kwargs.get('z', 0)

		self.calibrated = False 

		super(GPS, self).__init__(**kwargs)

		
		# self.measure()

	def set_zero(self, *args, **kwargs):
		self.calibrated = True 
		if (len(args) is 0) and (len(kwargs) is 0):
			i = 0;
			for key in self._measurementList:
				self.gps_zero[ key ] = self.Z.item(i)
				i += 1
		else:
			if ( len(args) is 1 ) and ( type(args[0]) == type(dict()) ) :
				set_zero( **args[0] )
			elif len(args) == 3:
				i = 0
				for key in self._measurementList:
					self.gps_zero[ key ] = args[i] 
					i += 1
			else:
				for key, value in kwargs.items():
					self.gps_zero[ key ] = value

	def map(self, **kwargs):
		for key, value in kwargs.items():
			self.Residual[self._measurementList.index(key)] = kwargs.get(key)

	def measure(self, **kwargs):
		cartesian = gps_transforms.gps2cart( **kwargs )

		for key, value in cartesian.items():
			cartesian[key] = value - self.gps_zero[key]

		super(GPS, self).measure(**cartesian)

class Camera(Sensor, object):
	"""docstring for Camera"""
	def __init__(self, **kwargs):
		super(Camera, self).__init__()

class Gyroscope(Sensor, object):
	"""docstring for Gyroscope"""
	def __init__(self, roll = 0, pitch = 0, yaw = 0, **kwargs):
		self._measurementList = ['roll', 'pitch', 'yaw']
		self._stateList = ['position.roll', 'position.pitch', 'position.yaw']

		self.roll = kwargs.get('roll', 0)
		self.pitch = kwargs.get('pitch', 0)
		self.yaw = kwargs.get('yaw', 0)

		super(Gyroscope, self).__init__(**kwargs)

class Accelerometer(Sensor, object):
	"""docstring for Accelerometer"""
	def __init__(self, **kwargs):
		self._measurementList = ['roll', 'pitch']
		self._stateList = ['position.roll', 'position.pitch']
		
		self.ax = kwargs.get('ax', 0)
		self.ay = kwargs.get('ay', 0)
		self.az = kwargs.get('az', 0)

		super(Accelerometer, self).__init__(**kwargs)

class Magnetomer(Sensor, object):
	"""docstring for Magnetomer"""
	def __init__(self, **kwargs):
		self._measurementList = ['yaw']
		self._stateList = ['position.yaw']

		self.Hx = kwargs.get('Hx', 0)
		self.Hy = kwargs.get('Hy', 0)
		self.Hz = kwargs.get('Hz', 0)
		super(Magnetomer, self).__init__(**kwargs)
		
def yaw_test():
	yaw_sensor = DummyYaw(yaw = pi/2 )
	print yaw_sensor.yaw 
	print yaw_sensor.Z 
	print yaw_sensor.Residual

	yaw_sensor.map(yaw = 1.56)
	yaw_sensor.measure(yaw = 1.57)
	print yaw_sensor.Z 
	print yaw_sensor.Residual
	print yaw_sensor.Jacobian

def gps2_test():

	gps = GPS(latitude = 49.8999999626, altitude = 0.242652171948, longitude = 8.89999939638) 
	print gps.get_measurementList()
	print gps.latitude, gps.longitude, gps.altitude

	gps.map(x = 0, y = 0, z = 0)
	print gps.Residual
	gps.map(x = 0, y = 0, z = 1)
	print gps.Residual
	gps.map(x = 1, y = 0, z = 0)
	print gps.Residual
	
	"""
	for item in gps:
		print item
	print gps.Z 
	gps.measure(latitude = 1, altitude = 12, longitude = -4)
	print gps.Z
	gps.set_zero()
	print gps.gps_zero
	gps.set_zero( latitude = 2., altitude = 1., longitude = 125. )
	print gps.gps_zero
	print 'latitude', gps.latitude



	print gps.Z
	print gps.Jacobian

	print 'Z', gps.Z
	print 'R', gps.Residual

	print gps.map(x = 0, y = 1, z = 3)

	# print 'Z', gps.Z
	print 'R', gps.Residual

	print 'longitude', gps.longitude
	print gps.properties
	print gps == 'GPS'
	# test(Gyroscope())
	# test(Accelerometer())
	"""

def gps():
	gps = GPS(x = 1, y = 2, z = 0, latitude = 45* pi/180.0, altitude = 0, longitude = 20* pi/180.0) 
	print gps.Z 
	gps.measure(latitude = 45 * pi/180.0, altitude = 1, longitude = 20* pi/180.0)
	print gps.Z 
	print gps.Jacobian
	print gps.Covariance

	gps.map(x = 1, y = 2, z = 3)


def main():
	# gps_test()
	yaw_test()
	gps()

	
if __name__ == '__main__': main()