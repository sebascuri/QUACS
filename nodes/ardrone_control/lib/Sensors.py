#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi, cos, sin, sqrt, atan2, asin
import numpy as np 
import numpy.matlib as matlib
import utm
# import gps_transforms

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


		There is a list that maps Z (measurement vector) and measurement properties named measurementList

	"""
	def __init__(self, **kwargs):
		# super(Sensor, self).__init__( **kwargs )

		self.properties = dict( measurementList = kwargs.get('measurementList', list() ),
			stateList = kwargs.get('stateList', list() ) )

		for attribute in self.measurementList:
			setattr(self, attribute, kwargs.get(attribute, 0.) )

		for key in self.stateList:
			attribute = key.split('.')[1]
			setattr( self, attribute, kwargs.get(attribute, 0.) )

		self.properties.update(
			Z = kwargs.get('Z', matlib.zeros( [len(self), 1] )),
			StateMap = kwargs.get('StateMap', matlib.zeros( [len(self) , 1] )),
			Jacobian = kwargs.get('Jacobian', matlib.eye( len(self) ) ),
			Covariance = kwargs.get('Covariance', 0.3 * matlib.eye( len(self) )),
			)

		# self.Z = kwargs.get('Z', matlib.zeros( [len(self), 1] ))
		# self.StateMap = kwargs.get('StateMap', matlib.zeros( [len(self) , 1] ))
		# self.Jacobian = kwargs.get('Jacobian', matlib.eye( len(self) ) )
		# self.Covariance = kwargs.get('Covariance', 0.3 * matlib.eye( len(self) ))

		self.set_Z()
		
	def __len__(self):

		return np.size(self.measurementList) 

	def __eq__(self, data):

		return str(self) == data

	def measure(self, **kwargs):
		""" input sensor values and assing them to state Vector"""
		for key in self.stateList:
			attribute = key.split('.')[1]
			setattr( self, attribute, kwargs.get(attribute, getattr(self, attribute) ) ) #input new value or stay with same value

		self.set_Z()

	def map(self, **kwargs):
		""" map state variables from Quadrotor and returns a vector of what it should have been measured """
		i = 0
		for key in self.stateList:
			attribute = key.split('.')[1]
			self.StateMap[ i ] = kwargs.get(attribute, self.StateMap[ i ])
			i += 1

		"""
		for key, value in kwargs.items():
			for name in self.stateList:
				attribute = name.split('.')[1]
				if attribute == key:
					self.StateMap[ self.stateList.index(name) ] = value
		"""

	def get_measurementList(self):

		return self.measurementList

	def set_Z(self):
		i = 0
		for key in self.stateList:
			attribute = key.split('.')[1]
			self.Z[i] = getattr(self, attribute)
			i += 1

	@property 
	def Z(self):
		return self.properties.get('Z', matlib.zeros( [len(self), 1] ) )
	@Z.setter 
	def Z(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( matlib.zeros(1) ):
				self.properties['Z'] = arg 
			elif type(arg) == type( dict() ):
				for key, value in arg.items():
					self.Z[ self.stateList.index(key) ][0] =  value

		for key, value in kwargs.items():
			self.Z[ self.stateList.index(key) ][0] =  value

	@Z.deleter
	def Z(self):
		del self.properties['Z']

	@property 
	def StateMap(self):
		return self.properties.get('StateMap', matlib.zeros( [len(self), 1] ) )
	@StateMap.setter 
	def StateMap(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( matlib.zeros(1) ):
				self.properties['StateMap'] = arg 
			elif type(arg) == type( dict() ):
				for key, value in arg.items():
					self.StateMap[ self.stateList.index(key) ][0] =  value

		for key, value in kwargs.items():
			self.StateMap[ self.stateList.index(key) ][0] =  value

	@StateMap.deleter
	def StateMap(self):
		del self.properties['StateMap']

	@property
	def Jacobian(self):
		return self.properties.get('Jacobian', matlib.eye( len(self) ) )
	@Jacobian.setter
	def Jacobian(self, H):
		self.properties['Jacobian'] = H
	@Jacobian.deleter
	def Jacobian(self):
		del self.properties['Jacobian']	

	@property
	def Covariance(self):
		return self.properties.get('Covariance', 0.3 * matlib.eye( len(self) ) )
	@Covariance.setter
	def Covariance(self, R):
		self.properties['Covariance'] = R
	@Covariance.deleter
	def Covariance(self):
		del self.properties['Covariance']

	@property 
	def stateList(self):
		return self.properties.get('stateList', list())
	@stateList.setter 
	def stateList(self, stateList):
		self.properties['stateList'] = stateList
	@stateList.deleter
	def stateList(self):
		del self.properties['stateList']

	@property 
	def measurementList(self):
		return self.properties.get('measurementList', list())
	@measurementList.setter 
	def measurementList(self, measurementList):
		self.properties['measurementList'] = measurementList
	@measurementList.deleter
	def measurementList(self):
		del self.properties['measurementList']

class DummyYaw(Sensor, object):
	measurementList = ['yaw']
	stateList = ['position.yaw']
	"""docstring for DummyYaw
	just a Dummy object used for Kalman Filtering Convergence. 
	Input is a Yaw, Output is the same Yaw 
	"""
	def __init__(self, **kwargs):
		super(DummyYaw, self).__init__(stateList = DummyYaw.stateList, measurementList = DummyYaw.measurementList, **kwargs)

		self.Covariance = matlib.zeros( (len(self), len(self)) )

	@property 
	def yaw(self):
		return self.properties.get('yaw', 0.0)
	@yaw.setter 
	def yaw(self, yaw):
		self.properties['yaw'] = yaw
	@yaw.deleter
	def yaw(self):
		del self.properties['yaw']

class GPS(Sensor, object):
	"""docstring for GPS 
	x is easting
	y is northing
	z is altitude 

	the gps_zero['yaw'] is a rotation around Z between northing, easting to global xyz.
	"""
	measurementList = ['latitude', 'longitude', 'altitude']
	stateList = ['position.x', 'position.y', 'position.z']
	def __init__(self, **kwargs):
		super(GPS, self).__init__(stateList = GPS.stateList, measurementList = GPS.measurementList, **kwargs)
		
		self.Covariance= 0.4 * matlib.eye( len(self.measurementList) )

		easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )

		self.gps_zero = dict( x = easting, y = northing, z = self.altitude, yaw = 0)

		self.calibrated = False 

	def set_zero(self, *args, **kwargs):
		if (len(args) is 0) and (len(kwargs) is 0):
			i = 0;
			for key in self.measurementList:
				self.gps_zero[ key ] = self.Z.item(i)
				i += 1
		else:
			if ( len(args) is 1 ) and ( type(args[0]) == type(dict()) ) :
				set_zero( **args[0] )
			elif len(args) == 3:
				i = 0
				for key in self.measurementList:
					self.gps_zero[ key ] = args[i] 
					i += 1
			else:
				for key, value in kwargs.items():
					self.gps_zero[ key ] = value

	def measure(self, *args, **kwargs):
		# get values
		if len(args) == 2: # only latitude and longitude
			self.altitude = utm.conversion.R 
			self.latitude = args[0]
			self.longitude = args[1]
		elif len(args) == 3: # latitude, longitude and altitude, ordered
			i = 0
			for attribute in self.measurementList:
				setattr(self, attribute, args[i])
				i += 1

		elif len(args) == 1:
			arg = args[0]
			if type(arg) == dict():
				self.measure(**args[0])
			else: # latitude, longitude and altitude, ordered inside a list or tuple
				i = 0 
				for attribute in self.measurementList:
					setattr(self, attribute, arg[i])
					i += 1

		for key, value in kwargs.items():
			if key in self.measurementList:
				setattr(self, key, value)

		# transform latitude longitude to easting, northing
		easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )

		# set attributes and Z vector
		super(GPS, self).measure(x = easting - self.gps_zero['x'], 
			y = northing - self.gps_zero['y'], 
			z = self.altitude - self.gps_zero['z'] )

	@property 
	def latitude(self):
		return self.properties.get('latitude', list())
	@latitude.setter 
	def latitude(self, latitude):
		self.properties['latitude'] = latitude
	@latitude.deleter
	def latitude(self):
		del self.properties['latitude']	

	@property 	
	def longitude(self):
		return self.properties.get('longitude', list())
	@longitude.setter 
	def longitude(self, longitude):
		self.properties['longitude'] = longitude
	@longitude.deleter
	def longitude(self):
		del self.properties['longitude']

	@property 
	def altitude(self):
		return self.properties.get('altitude', list())
	@altitude.setter 
	def altitude(self, altitude):
		self.properties['altitude'] = altitude
	@altitude.deleter
	def altitude(self):
		del self.properties['altitude']

	@property 
	def x(self):
		return self.properties.get('x', 0.0)
	@x.setter 
	def x(self, x):
		self.properties['x'] = x
	@x.deleter
	def x(self):
		del self.properties['x']

	@property 
	def y(self):
		return self.properties.get('y', 0.0)
	@y.setter 
	def y(self, y):
		self.properties['y'] = y
	@y.deleter
	def y(self):
		del self.properties['y']

	@property 
	def z(self):
		return self.properties.get('z', 0.0)
	@z.setter 
	def z(self, z):
		self.properties['z'] = z
	@z.deleter
	def z(self):
		del self.properties['z']

class Camera(Sensor, object):
	"""docstring for Camera"""
	def __init__(self, **kwargs):
		super(Camera, self).__init__()

class Gyroscope(Sensor, object):
	"""docstring for Gyroscope
	Reads roll, pitch and yaw velocities as input as assing them to outputs. 
	"""
	['roll', 'pitch', 'yaw']
	['velocity.roll', 'velocity.pitch', 'velocity.yaw']
	def __init__(self, roll = 0, pitch = 0, yaw = 0, **kwargs):
		super(Gyroscope, self).__init__(stateList = Gyroscope.stateList, measurementList = Gyroscope.measurementList, **kwargs)
		self.Covariance= 0.4 * matlib.eye( len(self.measurementList) )

	@property 
	def yaw(self):
		return self.properties.get('yaw', 0.0)
	@yaw.setter 
	def yaw(self, yaw):
		self.properties['yaw'] = yaw
	@yaw.deleter
	def yaw(self):
		del self.properties['yaw']

	@property 
	def pitch(self):
		return self.properties.get('pitch', 0.0)
	@pitch.setter 
	def pitch(self, pitch):
		self.properties['pitch'] = pitch
	@pitch.deleter
	def pitch(self):
		del self.properties['pitch']

	@property 
	def roll(self):
		return self.properties.get('roll', 0.0)
	@roll.setter 
	def roll(self, roll):
		self.properties['roll'] = roll
	@roll.deleter
	def roll(self):
		del self.properties['roll']

class Accelerometer(Sensor, object):
	"""docstring for Accelerometer
	Reads ax, ay and az and calculates roll and pitch of the vehicle. 
	"""
	measurementList = ['ax', 'ay', 'az']
	stateList = ['position.roll', 'position.pitch']
	def __init__(self, **kwargs):
		super(Accelerometer, self).__init__(stateList = Accelerometer.stateList, measurementList = Accelerometer.measurementList, **kwargs)

	def measure(self, *args, **kwargs):
		for key, value in kwargs.items():
			if key in self.measurementList:
				setattr(self, key, value)

		if len(args) == 1: 
			if type(arg) == dict():
				self.measure(**args[0])
			else: # ax, ay, az, ordered inside a list or tuple
				i = 0 
				for attribute in self.measurementList:
					setattr(self, attribute, arg[i])
					i += 1
		elif len(args) == 3: # ax, ay, az, ordered
			i = 0
			for attribute in self.measurementList:
				setattr(self, attribute, args[i])
				i += 1

		if self.az < 0:
			self.roll = atan2(- self.ay, -self.az )
			self.pitch = asin( self.ax / sqrt(self.ax ** 2 + self.ay ** 2 + self.az ** 2) )
		else:
			self.roll = atan2(self.ay, self.az)
			self.pitch = asin( -self.ax / sqrt(self.ax ** 2 + self.ay ** 2 + self.az ** 2) )

	@property 
	def pitch(self):
		return self.properties.get('pitch', 0.0)
	@pitch.setter 
	def pitch(self, pitch):
		self.properties['pitch'] = pitch
	@pitch.deleter
	def pitch(self):
		del self.properties['pitch']

	@property 
	def roll(self):
		return self.properties.get('roll', 0.0)
	@roll.setter 
	def roll(self, roll):
		self.properties['roll'] = roll
	@roll.deleter
	def roll(self):
		del self.properties['roll']

	@property 
	def ax(self):
		return self.properties.get('ax', 0.0)
	@ax.setter 
	def ax(self, ax):
		self.properties['ax'] = ax
	@ax.deleter
	def ax(self):
		del self.properties['ax']

	@property 
	def ay(self):
		return self.properties.get('ay', 0.0)
	@ay.setter 
	def ay(self, ay):
		self.properties['ay'] = ay
	@ay.deleter
	def ay(self):
		del self.properties['ay']

	@property 
	def az(self):
		return self.properties.get('az', 0.0)
	@az.setter 
	def az(self, az):
		self.properties['az'] = az
	@az.deleter
	def az(self):
		del self.properties['az']

class Magnetomer(Sensor, object):
	"""docstring for Magnetomer
	Reads Hx, Hy and Hz and calculates global yaw of the vehicle. 
	"""
	measurementList = ['Hx', 'Hy', 'Hz']
	stateList = ['position.yaw']
	def __init__(self, **kwargs):
		super(Magnetomer, self).__init__(stateList = Magnetomer.stateList, measurementList = Magnetomer.measurementList, **kwargs)
	
	def set_pitchroll(self, *args, **kwargs):
		if len(args) == 2:
			self.pitch = args[0]
			self.roll = args[1]
		elif len(args) == 1:
			arg = args[0]
			if type(arg) == dict():
				return set_pitchroll(**arg)
			else:
				self.pitch = arg[0]
				self.roll = arg[1]

		for key, value in kwargs.items():
			setattr(self, key, value)

	def measure(self, *args, **kwargs):
		for key, value in kwargs.items():
			if key in self.measurementList:
				setattr(self, key, value)

		if len(args) == 1: 
			if type(arg) == dict():
				self.measure(**args[0])
			else: # Hx, Hy, Hz ordered inside a list or tuple
				i = 0 
				for attribute in self.measurementList:
					setattr(self, attribute, arg[i])
					i += 1
		elif len(args) == 3: # Hx, Hy, Hz ordered
			i = 0
			for attribute in self.measurementList:
				setattr(self, attribute, args[i])
				i += 1

		self.yaw = atan2( self.Hz * sin( self.roll ) - self.Hy * cos( self.roll ), 
			self.Hx * cos( self.pitch ) + (self.Hy * sin( self.roll ) + self.Hz * cos( self.roll ) ) * sin( self.pitch ) )

		super(GPS, self).measure(yaw = self.yaw)

	@property 
	def Hx(self):
		return self.properties.get('Hx', 0.0)
	@Hx.setter 
	def Hx(self, Hx):
		self.properties['Hx'] = Hx
	@Hx.deleter
	def Hx(self):
		del self.properties['Hx']

	@property 
	def Hy(self):
		return self.properties.get('Hy', 0.0)
	@Hy.setter 
	def Hy(self, Hy):
		self.properties['Hy'] = Hy
	@Hy.deleter
	def Hy(self):
		del self.properties['Hy']

	@property 
	def Hz(self):
		return self.properties.get('Hz', 0.0)
	@Hz.setter 
	def Hz(self, Hz):
		self.properties['Hz'] = Hz
	@Hz.deleter
	def Hz(self):
		del self.properties['Hz']

	@property 
	def yaw(self):
		return self.properties.get('yaw', 0.0)
	@yaw.setter 
	def yaw(self, yaw):
		self.properties['yaw'] = yaw
	@yaw.deleter
	def yaw(self):
		del self.properties['yaw']

	@property 
	def pitch(self):
		return self.properties.get('pitch', 0.0)
	@pitch.setter 
	def pitch(self, pitch):
		self.properties['pitch'] = pitch
	@pitch.deleter
	def pitch(self):
		del self.properties['pitch']

	@property 
	def roll(self):
		return self.properties.get('roll', 0.0)
	@roll.setter 
	def roll(self, roll):
		self.properties['roll'] = roll
	@roll.deleter
	def roll(self):
		del self.properties['roll']


def yaw_test():
	yaw_sensor = DummyYaw(yaw = pi/2 )
	print yaw_sensor.yaw 
	print yaw_sensor.Z 
	print yaw_sensor.StateMap
	print yaw_sensor.Covariance

	yaw_sensor.map(yaw = 1.56)
	yaw_sensor.measure(yaw = 1.57)
	print yaw_sensor.Z 
	print yaw_sensor.StateMap
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
	gps = GPS(latitude = 45, altitude = utm.conversion.R, longitude = 20) 

	print gps.get_properties()

	print gps.x, gps.y, gps.z 
	gps.measure( latitude = 45, altitude = utm.conversion.R + 1., longitude = 20)
	print gps.x, gps.y, gps.z  
	gps.measure( latitude = 44, altitude = utm.conversion.R, longitude = 20)
	print gps.x, gps.y, gps.z 
	print (gps.x**2 + gps.y**2 )**0.5
	print gps.Z 


	gps.measure( latitude = 45, altitude = utm.conversion.R, longitude = 21)
	print gps.x, gps.y, gps.z 
	print (gps.x**2 + gps.y**2 )**0.5
	print gps.Z 
	# gps.measure(latitude = 45 * pi/180.0, altitude = 1, longitude = 20* pi/180.0)
	
	print gps.Jacobian
	print gps.Covariance

	gps.map(x = 1, y = 2, z = 3)

	print gps.StateMap

	print gps.get_properties()


def main():
	#gps_test()
	yaw_test()
	gps()

	
if __name__ == '__main__': main()