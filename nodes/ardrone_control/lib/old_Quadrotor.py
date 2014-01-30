#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi
class BasicObject(object):
	"""docstring for BasicObject"""
	def __init__(self, **kwargs):
		super(BasicObject, self).__init__()
		for key, value in kwargs.items():
			self.properties[key] = value
	def __str__(self):
		return str(self.properties)

	def get_properties(self):
		return self.properties;

	def get_property(self, key):
		return self.properties.get(key, None)

class SixDofObject(BasicObject, object):
	"""docstring for SixDofObject:
	This object has properties for each of the 6 degrees of freedom"""
	def __init__(self, **kwargs):
		self.properties = dict(x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0)
		super(SixDofObject, self).__init__(**kwargs)
		
	@property 
	def x(self):
		return self.properties.get('x', 0)
	@x.setter 
	def x(self, x):
		self.properties['x'] = x
	@x.deleter
	def x(self):
		del self.properties['x']

	@property 
	def y(self):
		return self.properties.get('y', 0)
	@y.setter 
	def y(self, y):
		self.properties['y'] = y
	@y.deleter
	def y(self):
		del self.properties['y']

	@property 
	def z(self):
		return self.properties.get('z', 0)
	@z.setter 
	def z(self, z):
		self.properties['z'] = z
	@z.deleter
	def z(self):
		del self.properties['z']

	@property 
	def pitch(self):
		return self.properties.get('pitch', 0)
	@pitch.setter 
	def pitch(self, pitch):
		self.properties['pitch'] = pitch
	@pitch.deleter
	def pitch(self):
		del self.properties['pitch']

	@property 
	def roll(self):
		return self.properties.get('roll', 0)
	@roll.setter 
	def roll(self, roll):
		self.properties['roll'] = roll
	@roll.deleter
	def roll(self):
		del self.properties['roll']

	@property 
	def yaw(self):
		return self.properties.get('yaw', 0)
	@yaw.setter 
	def yaw(self, yaw):
		self.properties['yaw'] = yaw
	@yaw.deleter
	def yaw(self):
		del self.properties['yaw']
		
class Position(SixDofObject, object):
	"""docstring for Position"""
	def __init__(self, **kwargs):
		super(Position, self).__init__(**kwargs)
			
class Velocity(SixDofObject, object):
	"""docstring for Velocity"""
	def __init__(self, **kwargs):
		super(Velocity, self).__init__(**kwargs)

class Acceleration(SixDofObject, object):
	"""docstring for Acceleration"SixDofObject, """
	def __init__(self, **kwargs):
		super(Acceleration, self).__init__(**kwargs)

class State(object):
	"""docstring for State"""
	def __init__(self, state = 0):
		super(State, self).__init__()
		self._STATES= {0 : 'Unknown', 1 : 'Inited', 2 : 'Landed', 3 : 'Flying', 4 : 'Hovering', 5 : 'Test', 6 : 'Taking off', 7 : 'Flying', 8 : 'Landed', 9 : 'Looping'}

		self.state = state
		
	def __str__(self):
		return str(self._state)

	@property 
	def state(self):
		return self._state 
	@state.setter
	def state(self, state):
		if (state in self._STATES.values()) or (state in self._STATES.keys()):
			if type(state) == int:
				self._state = self._STATES[state]
			else:
				self._state = state
		else:
			self._state = self._STATES[0]
			print 'State not recognized, setting it to Unkown'

	@state.deleter
	def state(self):
		del self._state

class Motor(BasicObject, object):
	"""docstring for Motors"""
	def __init__(self, **kwargs):
		super(Motor, self).__init__(**kwargs)
		if not 'self.properties' in locals():
			self.properties = dict()

	def __str__(self):
		return str(self.pwm)
		
	@property 
	def pwm(self):
		return self.properties.get('pwm', 0)
	@pwm.setter
	def pwm(self, pwm):
		self.properties['pwm'] = pwm

	@pwm.deleter
	def pwm(self):
		del self.properties['pwm']

class Quadrotor(BasicObject, object):
	"""docstring for Quadrotor"""
	def __init__(self, **kwargs):
		self.properties = dict()
		self.properties['position'] = SixDofObject()
		self.properties['velocity'] = SixDofObject()
		self.properties['acceleration'] = SixDofObject()
		self.properties['battery'] = 100
		self.properties['motors'] = [Motor(), Motor(), Motor(), Motor()]
		self.properties['state'] = State()
		super(Quadrotor, self).__init__(**kwargs)

	def __str__(self):
		return str(self.properties)

	@property 
	def position(self):
		return self.properties.get('position', None)
	@position.setter 
	def position(self, kwargs):
		for key, value in kwargs.items():
			setattr(self.position, key, value)
	@position.deleter
	def position(self):
		del self.properties['position']

	@property 
	def velocity(self):
		return self.properties.get('velocity', None)
	@velocity.setter 
	def velocity(self, kwargs):
		for key, value in kwargs.items():
			setattr(self.velocity, key, value)
	@velocity.deleter
	def velocity(self):
		del self.properties['velocity']

	@property 
	def acceleration(self):
		return self.properties.get('acceleration', None)
	@acceleration.setter 
	def acceleration(self, kwargs):
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
		
def state_test():
	print "State Test"
	state = State(state = 'Landed')
	print state

	state.state = 4
	print state

	state.state = -3
	print state

	state.state = 'Flying'
	print state

	state.state = 'Flyiiing'
	print state

def motor_test():
	print "Motor Test"
	motor = Motor()
	print motor
	print motor.pwm
	motor.pwm = 25
	print motor
	print motor.pwm
	
def sixdofobject_test():
	print "SixDofObject Test"
	position = SixDofObject()
	position.x = 2
	position.yaw = pi/4
	print position
	print position.x	
	print position.yaw

def quadrotor_test():
	print "Quadrotor Test"
	parrot = Quadrotor(battery = 80, position = Position(x = 2))
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

	parrot.position = dict(z = -3, pitch = 67)
	print parrot.position

	print parrot



def main():
	#state_test()
	#motor_test()
	#sixdofobject_test()
	#quadrotor_test()
	



	"""
	parrot = Quadrotor( position = Position(), velocity = Velocity())
	print type(parrot.properties['velocity'])
	parrot.velocity = {'yaw':2, 'roll':3}

	print parrot.velocity.yaw
	print parrot.velocity

	parrot.velocity.yaw = 2
	print parrot.velocity.yaw
	print parrot.velocity
	print parrot.properties['velocity'].properties
	"""

	#print parrot.velocity.yaw




if __name__ == '__main__': main()