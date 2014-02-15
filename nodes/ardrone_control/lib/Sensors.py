#!/usr/bin/env python

from math import pi, cos, sin, sqrt, atan2, asin
import numpy as np 
import numpy.matlib as matlib

import utm
import rospy;


# import gps_transforms

from BasicObject import BasicObject, SixDofObject, Quaternion
from Filter import ExtendedKalmanFilter, MagdwickFilter, MahoneyFilter

from ROS import ROS_Object

try:
	import tf;
except ImportError:
	import roslib; roslib.load_manifest('ardrone_control')
	import tf;



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
		super(Sensor, self).__init__( **kwargs )

		self.properties.update( measurementList = kwargs.get('measurementList', list() ),
			stateList = kwargs.get('stateList', list() ) )

		for attribute in self.measurementList:
			setattr(self, attribute, kwargs.get(attribute, 0.) )

		for key in self.stateList:
			attribute = key.split('.')[1]
			setattr( self, attribute, kwargs.get(attribute, 0.) )

		self.properties.update(
			Z = kwargs.get('Z', matlib.zeros( [len(self), 1] )),
			StateMap = kwargs.get('StateMap', matlib.zeros( [len(self) , 1] )),
			MeasurementJacobian = kwargs.get('MeasurementJacobian', matlib.eye( len(self) ) ),
			MeasurementCovariance = kwargs.get('MeasurementCovariance', 0.3 * matlib.eye( len(self) )),
			)

		# self.Z = kwargs.get('Z', matlib.zeros( [len(self), 1] ))
		# self.StateMap = kwargs.get('StateMap', matlib.zeros( [len(self) , 1] ))
		# self.MeasurementJacobian = kwargs.get('MeasurementJacobian', matlib.eye( len(self) ) )
		# self.MeasurementCovariance = kwargs.get('MeasurementCovariance', 0.3 * matlib.eye( len(self) ))

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

	def get_state(self):
		state_dict = dict( )
		for key in self.stateList:
			attribute = key.split('.')[1]
			state_dict[attribute] = getattr(self, attribute)
		return state_dict

	def get_measurement(self):
		measure_dict = dict( )
		for key in self.properties.keys():
			if key in self.measurementList:
				measure_dict[key] = getattr(self, key)

		return measure_dict

	def get_measurementvector(self):
		l = []
		for measure in self.measurementList:
			l.append(getattr(self, measure))
		return np.array(l).transpose()

	def get_measurementList(self):

		return self.measurementList

	def set_Z(self):
		i = 0
		for key in self.measurementList:
			# attribute = key.split('.')[1]
			self.Z[i] = getattr(self, key)
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
	def MeasurementJacobian(self):
		return self.properties.get('MeasurementJacobian', matlib.eye( len(self) ) )
	@MeasurementJacobian.setter
	def MeasurementJacobian(self, H):
		self.properties['MeasurementJacobian'] = H
	@MeasurementJacobian.deleter
	def MeasurementJacobian(self):
		del self.properties['MeasurementJacobian']	

	@property
	def MeasurementCovariance(self):
		return self.properties.get('MeasurementCovariance', 0.3 * matlib.eye( len(self) ) )
	@MeasurementCovariance.setter
	def MeasurementCovariance(self, R):
		self.properties['MeasurementCovariance'] = R
	@MeasurementCovariance.deleter
	def MeasurementCovariance(self):
		del self.properties['MeasurementCovariance']

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

		self.MeasurementCovariance = matlib.zeros( (len(self), len(self)) )

		# self.x = 0
		# self.y = 0
		# self.z = 0
		# self.w = 1 

	def measure(self, imu_raw):
		q = imu_raw.orientation
		euler = tf.transformations.euler_from_quaternion( (q.x, q.y, q.z, q.w), axes = 'sxyz' )

		super(DummyYaw, self).measure(yaw = euler[2] )

	@property 
	def yaw(self):
		return self.properties.get('yaw', 0.0)
	@yaw.setter 
	def yaw(self, yaw):
		self.properties['yaw'] = yaw
	@yaw.deleter
	def yaw(self):
		del self.properties['yaw']

class GPS(Sensor, ROS_Object, object):
	"""docstring for GPS 
	x is easting
	y is northing
	z is altitude 

	the gps_zero['yaw'] is a rotation around Z between northing, easting to global xyz.
	"""
	from geometry_msgs.msg import Vector3Stamped; global Vector3Stamped
	import tf; global tf 
	measurementList = ['latitude', 'longitude', 'altitude']
	stateList = ['position.x', 'position.y', 'position.z']
	def __init__(self, **kwargs):
		super(GPS, self).__init__(stateList = GPS.stateList, measurementList = GPS.measurementList, **kwargs)
		
		self.MeasurementCovariance= 0.4 * matlib.eye( len(self.measurementList) )

		easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )

		self.gps_zero = dict( x = easting, y = northing, z = self.altitude, yaw = pi/2)

		self.tf_broadcaster = dict( 
			enu_gps = tf.TransformBroadcaster(), 
			global_gps = tf.TransformBroadcaster()
			)
		
		self.Broadcast()

		self.calibrated = False


	def set_zero(self, fix_zero):
		easting, northing, number, letter = utm.from_latlon( fix_zero.latitude, fix_zero.longitude )

		self.gps_zero['x'] = easting
		self.gps_zero['y'] = northing 
		self.gps_zero['z'] = fix_zero.altitude
	
	def set_zero_yaw(self, yaw):
		self.gps_zero['yaw'] = yaw  

	def measure(self, fix_data):
		# set measurements 
		for key in self.measurementList:
			setattr(self, key, getattr(fix_data, key) )

		# transform latitude longitude to easting, northing
		easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )

		enu = Vector3Stamped( )
		enu.vector.x = easting - self.gps_zero['x']
		enu.vector.y = northing - self.gps_zero['y']
		enu.vector.z = fix_data.altitude - self.gps_zero['z']
		enu.header.frame_id = "/enu_gps"


		
		try:
			global_gps = self.tfListener.transformVector3("/global_gps", enu)
			super(GPS, self).measure(
				x = global_gps.vector.x, 
				y = global_gps.vector.y, 
				z = global_gps.vector.z
			)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			super(GPS, self).measure(
				x = enu.vector.x, 
				y = enu.vector.y, 
				z = enu.vector.z
			) 


		self.Broadcast()
		# set attributes and Z vector
		

	def Broadcast(self):
		# print "Broadcasting"
		time = rospy.Time.now()

		self.tf_broadcaster['global_gps'].sendTransform( (self.x, self.y, self.z ), 
			tf.transformations.quaternion_from_euler(0, 0, self.gps_zero['yaw']), 
			time,
			"/global_gps",
			"/nav"
			)

		self.tf_broadcaster['enu_gps'].sendTransform( (self.x, self.y, self.z ), 
			tf.transformations.quaternion_from_euler(0, 0, 0), 
			time,
			"/enu_gps",
			"/nav"
			)

	@property 
	def latitude(self):
		return self.properties.get('latitude', 0.0 )
	@latitude.setter 
	def latitude(self, latitude):
		self.properties['latitude'] = latitude
	@latitude.deleter
	def latitude(self):
		del self.properties['latitude']	

	@property 	
	def longitude(self):
		return self.properties.get('longitude', 0.0)
	@longitude.setter 
	def longitude(self, longitude):
		self.properties['longitude'] = longitude
	@longitude.deleter
	def longitude(self):
		del self.properties['longitude']

	@property 
	def altitude(self):
		return self.properties.get('altitude', utm.conversion.R )
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
	measurementList = ['roll', 'pitch', 'yaw']
	stateList = ['velocity.roll', 'velocity.pitch', 'velocity.yaw']
	Angles_MAP = dict(
		roll = 'x',
		pitch = 'y',
		yaw = 'z' 
		)
	def __init__(self, roll = 0, pitch = 0, yaw = 0, **kwargs):
		super(Gyroscope, self).__init__(stateList = Gyroscope.stateList, measurementList = Gyroscope.measurementList, **kwargs)
		self.MeasurementCovariance= 0.4 * matlib.eye( len(self.measurementList) )

	def measure(self, imu_raw):
		for measure in self.measurementList:
			setattr(self, measure, getattr(imu_raw.angular_velocity, self.Angles_MAP[measure] ) )

		self.set_Z()

	def get_quaternion(self):
		return (self.roll, self.pitch, self.yaw, 0)



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

	def measure(self, imu_raw):
		for measure in self.measurementList:
			setattr(self, measure, getattr(imu_raw.linear_acceleration, measure[1] ) ) 

		g = sqrt(self.ax**2 + self.ay**2 + self.az**2)
		self.roll = atan2(-self.ax, self.az)
		self.pitch = -asin(self.ay/g)

		self.set_Z()



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

class RangeSensor(BasicObject, object):
	"""docstring for RangeSensor"""
	def __init__(self, **kwargs):
		super(RangeSensor, self).__init__()
		self.max_range = kwargs.get('max_range', 3000.0)
		self.min_range = kwargs.get('min_range', 0.0)
		self.range = kwargs.get('range', 0.0)

		self.min_safe = kwargs.get('min_safe', None)


	def measure(self, range_data):
		for key in self.properties.keys():
			setattr(self, key, getattr(range_data, key) )

	def isFar(self):
		return self.range >= self.max_range 

	def isNear(self):
		return self.range <= self.min_range or self.range <= self.min_safe 


	@property 
	def range(self):
		return self.properties.get('range', None)
	@range.setter 
	def range(self, new_range):
		self.properties['range'] = new_range
	@range.deleter
	def range(self):
		del self.properties['range']

	@property 
	def max_range(self):
		return self.properties.get('max_range', None)
	@max_range.setter 
	def max_range(self, max_range):
		self.properties['max_range'] = max_range
	@max_range.deleter
	def max_range(self):
		del self.properties['max_range']

	@property 
	def min_range(self):
		return self.properties.get('min_range', None)
	@min_range.setter 
	def min_range(self, min_range):
		self.properties['min_range'] = min_range
	@min_range.deleter
	def min_range(self):
		del self.properties['min_range']

class IMU(BasicObject, object):
	"""docstring for IMU"""
	measurementList = ['roll', 'pitch', 'yaw']
	def __init__(self, **kwargs):
		super(IMU, self).__init__(**kwargs)
		self.sensors = dict(
			dummy_yaw = DummyYaw(), 
			gyroscope = Gyroscope(),
			accelerometer = Accelerometer()
			)

	def measure(self, imu_raw):
		for sensor in self.sensors.values():
			sensor.measure(imu_raw)

	@property 
	def sensors(self):
		return self.properties.get('sensors', None)
	@sensors.setter 
	def sensors(self, sensors):
		self.properties['sensors'] = sensors
	@sensors.deleter
	def sensors(self):
		del self.properties['sensors']

def yaw_test():
	yaw_sensor = DummyYaw(yaw = pi/2 )
	print yaw_sensor.yaw 
	print yaw_sensor.Z 
	print yaw_sensor.StateMap
	print yaw_sensor.MeasurementCovariance

	yaw_sensor.map(yaw = 1.56)
	yaw_sensor.measure(yaw = 1.57)
	print yaw_sensor.Z 
	print yaw_sensor.StateMap
	print yaw_sensor.MeasurementJacobian

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
	print gps.MeasurementJacobian

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
	import roslib; roslib.load_manifest('ardrone_control')
	import rospy;  global rospy 
	
	

	from sensor_msgs.msg import NavSatFix
	
	
	rospy.init_node('test')

	gps = GPS() 
	#print gps 

	m = NavSatFix()
	m.latitude = 45.0
	m.altitude = utm.conversion.R
	m.longitude = 20.0

	gps.set_zero(m) 

	gps.measure(m)
	print gps.get_state()
	print gps.Z 

	m.latitude = 45.1
	m.altitude = utm.conversion.R
	m.longitude = 20.0

	gps.measure(m)
	print gps.get_state()
	print gps.Z 

	gps.set_zero_yaw( 30 * pi/180 )
	gps.measure(m)
	print gps.get_state()
	print gps.get_measurement()
	print gps.Z 

	for i in range(10000):
		gps.measure(m)
		gps.Broadcast()

	rospy.spin()

def range_test():
	from sensor_msgs.msg import Range 
	sensor = RangeSensor()
	print sensor

	m = Range()
	m.range = 2000.0
	m.max_range = 3000.0
	m.min_range = 2.01

	sensor.min_safe = 500

	sensor.measure(m)

	print sensor 
	print sensor.isNear()
	print sensor.isFar()

	m.range = 4000
	sensor.measure(m)
	print sensor 
	print sensor.isNear()
	print sensor.isFar()

	m.range = -2000
	sensor.measure(m)
	print sensor 
	print sensor.isNear()
	print sensor.isFar()

	m.range = 300
	sensor.measure(m)
	print sensor 
	print sensor.isNear()
	print sensor.isFar()

def imu_test():
	g = 9.81
	from sensor_msgs.msg import Imu

	import roslib; roslib.load_manifest('ardrone_control')
	import rospy;  global rospy 
	import tf; global tf;
	# sensor = IMU()

	msg = Imu()
	msg.angular_velocity.z = 1.0
	msg.linear_acceleration.z = -9.81
	msg.linear_acceleration.x = -0.01
	msg.orientation.z = 1/sqrt(2)
	msg.orientation.w = 1/sqrt(2)
	# msg.linear_acceleration.z = g 
	# sensor.measure(msg)
	
	# for sensor in sensor.sensors.values():
	# 	print sensor.properties
	sensor = IMU()
	#sensor = IMU_Mahoney( Ts = 0.01)
	#sensor = IMU_Magdwick( Ts = 0.01)
	sensor.measure(msg)
	print sensor.sensors['accelerometer']
	print sensor.sensors['gyroscope']
	print sensor.sensors['dummy_yaw']
	# print sensor.X 
	# sensor.predict()
	# sensor.predict()
	# sensor.predict()
	# sensor.predict()
	# print sensor.X 

	# sensor.correct()

	# print sensor.get_quaternion()
	# print sensor.get_eulers()

def main():
	
	#gps_test()
	#yaw_test()
	# gps()
	#range_test()
	imu_test()

	
if __name__ == '__main__': main()