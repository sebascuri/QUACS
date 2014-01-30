#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi
from BasicObject import BasicObject, SixDofObject, Quaternion, Motor, State
import Process
import Sensors
# from EKF import EKF

class Quadrotor(BasicObject, object):
	"""docstring for Quadrotor"""
	def __init__(self, **kwargs):
		self.properties = dict()
		self.properties['position'] = SixDofObject()
		self.properties['orientation'] = Quaternion()
		self.properties['velocity'] = SixDofObject()
		self.properties['acceleration'] = SixDofObject()
		self.properties['battery'] = 100
		self.properties['motors'] = [Motor(), Motor(), Motor(), Motor()]
		self.properties['state'] = State()
		self.properties['processes'] = kwargs.get('processes', list() )
		self.properties['sensors'] = kwargs.get('sensors', list() )

		# self.properties['filters'] = [EKF(processes = self.processes, sensors = self.sensors)]

		super(Quadrotor, self).__init__(**kwargs)

	def predict( self ):
		for process in self.processes:
			changed = process.update( position = self.position, orientation = self.orientation, velocity = self.velocity, acceleration = self.acceleration )
			for attribute in changed.keys():
				setattr(self, attribute, changed[attribute])

			

		""" for filt in self.filters: 
			X, stateList = filt.predict( processes = self.processes )
			self.set_attributes(X, stateList)
		"""

	"""
	def measure(self, **kwargs ):
		for sensor in self.sensors:
			# measure
			measurements = dict()
			for item in sensor.get_measurementList( ) : 
				if item in kwargs.keys():
					measurements[item] = kwargs[item]
				
			sensor.measure( **measurements ) 
			# map 
			sensor.map(position = self.position, orientation = self.orientation, velocity = self.velocity, acceleration = self.acceleration )
			
		for filt in self.filters:
			X, stateList = filt.correct( sensors = self.sensors )
			self.set_attributes(X, stateList)
	""" 


	def set_attributes(self, X, stateList):
		i = 0
		for item in stateList:
			attribute, direction = item.split('.')
			setattr( getattr( self, attribute), direction, X.item(i) )
			i += 1



	@property 
	def position(self):
		return self.properties.get('position', None)
	@position.setter 
	def position(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( dict() ):
				for key, value in arg.items():
					setattr(self.position, key, value)
		for key, value in kwargs.items():
			setattr(self.position, key, value)
			
	@position.deleter
	def position(self):
		del self.properties['position']

	@property 
	def orientation(self):
		return self.properties.get('orientation', None)
	@orientation.setter 
	def orientation(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( dict() ):
				for key, value in arg.items():
					setattr(self.orientation, key, value)
		for key, value in kwargs.items():
			setattr(self.orientation, key, value)

	@orientation.deleter
	def orientation(self):
		del self.properties['orientation']

	@property 
	def velocity(self):
		return self.properties.get('velocity', None)
	@velocity.setter 
	def velocity(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( dict() ):
				for key, value in arg.items():
					setattr(self.velocity, key, value)
		for key, value in kwargs.items():
			setattr(self.velocity, key, value)
	@velocity.deleter
	def velocity(self):
		del self.properties['velocity']

	@property 
	def acceleration(self):
		return self.properties.get('acceleration', None)
	@acceleration.setter 
	def acceleration(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( dict() ):
				for key, value in arg.items():
					setattr(self.acceleration, key, value)
		for key, value in kwargs.items():
			setattr(self.acceleration, key, value)
	@acceleration.deleter
	def acceleration(self):
		del self.properties['acceleration']

	@property 
	def battery(self):
		return self.properties.get('battery', None)
	@battery.setter 
	def battery(self, value):
		self.properties['battery'] = value
	@battery.deleter
	def battery(self):
		del self.properties['battery']

	@property 
	def motors(self):
		return self.properties.get('motors', None)
	@motors.setter
	def motors(self, values):
		i = 0
		for motor in self.properties.get('motors'):
			motor.pwm = values[i]
			i+=1
	@motors.deleter
	def motors(self):
		del self.properties['motors']

	@property 
	def state(self):
		return self.properties.get('state', None)
	@state.setter 
	def state(self, value):
		self.properties['state'] = value
	@state.deleter
	def state(self):
		del self.properties['state']

	@property 
	def processes(self):
		return self.properties.get('processes', None)
	@processes.setter 
	def processes(self, processes):
		self.properties['processes'] = processes
	@processes.deleter
	def processes(self):
		del self.properties['processes']

	@property 
	def sensors(self):
		return self.properties.get('sensors', None)
	@sensors.setter 
	def sensors(self, value):
		self.properties['sensors'] = sensor
	@sensors.deleter
	def sensors(self):
		del self.properties['sensors']

	"""
	@property 
	def filters(self):
		return self.properties.get('filters', None)
	@filters.setter 
	def filters(self, value):
		self.properties['filters'] = sensor
	@filters.deleter
	def filters(self):
		del self.properties['filters']
	""" 


def quadrotor_test():
	print "Quadrotor Test"
	parrot = Quadrotor(battery = 80, position = SixDofObject(x = 2))
	print parrot.properties['battery']
	print parrot.battery

	parrot.state = 'Flying'
	print parrot.state

	for motor in parrot.motors:
		print motor
	parrot.motors = [50, 70, 20, 40]
	for motor in parrot.motors:
		print motor
	
	print parrot.velocity
	parrot.velocity.x = 2
	print parrot.velocity
	parrot.velocity = {'x': -3}
	print parrot.velocity
	parrot.velocity = dict(yaw = pi/2, pitch = 14, roll = -12)
	print parrot.velocity

	parrot.acceleration = dict(x = -3, y = 12)
	print parrot.acceleration 

	parrot.orientation = dict(x = 1, y = -2, w = 0.2)
	print parrot.orientation
	print parrot.orientation.w

	parrot.position = dict(z = -3, pitch = 67)
	print parrot.position

	print parrot

	print "Attribute Test"


	parrot.position = dict( x = 10, y = 100, z = 1000, pitch = 10000, roll = 100000, yaw = 1000000)
	for i in parrot.position:
		print i

	parrot.velocity = dict( x = 20, y = 200, z = 2000, pitch = 20000, roll = 200000, yaw = 2000000)
	for i in parrot.velocity:
		print i

	parrot.acceleration = dict( x = 30, y = 300, z = 3000, pitch = 30000, roll = 300000, yaw = 3000000)
	for i in parrot.acceleration:
		print i

	parrot.orientation = dict( x = 40, y = 400, z = 4000, w = 40000)
	for i in parrot.orientation:
		print i

	parrot.battery = 50
	print parrot.battery

	parrot.motors = [60, 61, 62, 63]
	for motor in parrot.motors:
		print motor

	parrot.sate = 'Hovering'
	print parrot.state



def main():
	#state_test()
	#motor_test()
	#sixdofobject_test()
	#quaternion_test()
	# quadrotor_test()
	#iterate_test()

	processes = [Process.XY_Odometry1(Ts = 1), Process.Z_Odometry2(Ts = 1) ]
	sensors = [ Sensors.GPS() ]

	parrot = Quadrotor(battery = 80, position = SixDofObject(x = 2, y = -2, yaw = -pi/4), velocity = SixDofObject(x = 1, y = 0), processes = processes, sensors = sensors)
	parrot.predict()
	print repr(parrot.position)
	parrot.predict()
	print repr(parrot.position)

	#print parrot.velocity.yaw




if __name__ == '__main__': main()