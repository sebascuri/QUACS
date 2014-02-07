#!/usr/bin/env python
#!/users/sebastiancuri/anaconda/bin/python

from math import pi, sqrt
try:
	import tf
except ImportError:
	pass


class BasicObject(object):
	"""docstring for BasicObject
	Object that has standard methods such as get_property or set_properties

	overrides or starts standard attributes such as str, rerp, len, iter and eq for future control

	init method starts or assigns to properties kwargs items.
	"""
	def __init__(self, **kwargs):
		super(BasicObject, self).__init__()

		if not hasattr(self, 'properties'):
			self.properties = dict()
		
		for key, value in kwargs.items():
			self.properties[key] = value
	
	def __str__(self):
		try:
			return str(self.properties) #str(vars(self)) 
		except AttributeError:
			return str(vars(self))
		
	def __repr__(self):

		return str( self.__class__.__name__) 

	def __len__(self):

		return len(self.properties)  # len(vars(self)) 

	def __iter__(self):
		for key, value in self.properties.items():
			yield key, value 

	def __eq__(self, data):

		return str(self) == data

	def get_properties(self):

		return self.properties;

	def get_property(self, key):

		return self.properties.get(key, None)

	def set_properties(self, *args, **kwargs):
		if len(args) == 1:
			if type(args[0]) == dict:
				self.set_properties(** args[0])

		for key, value in kwargs.items():
			setattr(self, key, value)

class SixDofObject(BasicObject, object):
	"""docstring for SixDofObject:
	This object has properties for each of the 6 degrees of freedom"""
	def __init__(self, **kwargs):
		self.properties = dict( 
			x = kwargs.get('x', 0.0), 
			y = kwargs.get('y', 0.0),
			z = kwargs.get('z', 0.0),
			yaw = kwargs.get('yaw', 0.0),
			pitch = kwargs.get('pitch', 0.0),
			roll = kwargs.get('roll', 0.0)
			)
		"""
		self.x = kwargs.get('x', 0.0)
		self.y = kwargs.get('y', 0.0)
		self.z = kwargs.get('z', 0.0)
		self.yaw = kwargs.get('yaw', 0.0)
		self.pitch = kwargs.get('pitch', 0.0)
		self.roll = kwargs.get('roll', 0.0)
		"""

		# self.properties = dict(x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0)
		# super(SixDofObject, self).__init__(**kwargs)
	
	# Object Properties
	
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
	
class Quaternion(BasicObject, object):
	"""docstring for Quaternion 
	Object that has x,y,z,w properties for each quaternion. 
	Has a method that normalizes and entry 
	Has a method that from input euler angles it assigns the quaternion
	"""
	def __init__(self, **kwargs):
		self.properties = dict( 
			x = kwargs.get('x', 0.0), 
			y = kwargs.get('y', 0.0), 
			z = kwargs.get('z', 0.0), 
			w = kwargs.get('w', 1.0) )

		self.normalize()
		# self.x = kwargs.get('x', 0.0)
		# self.y = kwargs.get('y', 0.0)
		# self.z = kwargs.get('z', 0.0)
		# self.w = kwargs.get('w', 1.0)
		# super(Quaternion, self).__init__(**kwargs)

	def set_euler(self, *args, **kwargs):
		
		if len(args) == 3:
			roll = args[0]
			pitch = args[1]
			yaw = args[2]
		elif len(args) == 1:
			if type(args) == dict:
				self.set_euler(**args[0])
			else:
				arg = args[0]
				roll = arg[0]
				pitch = arg[1]
				yaw = arg[2]
		else:
			roll = kwargs.get('roll')
			pitch = kwargs.get('pitch')
			yaw = kwargs.get('yaw')

		quaternion = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx')		
		self.x = quaternion[0]
		self.y = quaternion[1]
		self.z = quaternion[2]
		self.w = quaternion[3]

	def normalize(self):
		norm = 0
		for value in self.properties.values():
			norm += value ** 2

		norm = sqrt(norm)

		for key, value in self.properties.items():
			setattr(self, key, value/norm )

	# Object Properties

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

	@property 
	def w(self):
		return self.properties.get('w', 1.0)
	@w.setter 
	def w(self, w):
		self.properties['w'] = w
	@w.deleter
	def w(self):
		del self.properties['w']
						
class State(BasicObject, object):
	"""docstring for State 
	Object that can take a MAP of states and handles key and name interaction
	"""
	def __init__(self, state = 0):
		# super(State, self).__init__()
		self.properties = dict( state = state)
		self.set_state(state)
		
	def __str__(self):

		return str(self.state)

	def __eq__(self, data):
		if type(data) == str:
			return str(self.state) is data
		elif type(data) == int:
			return str(self.state) is self.MAP[data]

	def set_state(self, state):
		if type(state) == int and len(self.MAP) > state:
			self.state = self.MAP[state]
		elif state in self.MAP:
			self.state = state
		else:
			self.state = self.MAP[0] 
			# print 'State not recognized, setting it to Unkown'
		"""
		if (state in self.MAP.values()) or (state in self.MAP.keys()):
			if type(state) == int:
				self.state = self.MAP[state]
			else:
				self.state = state
		else:
			self.state = self.MAP[0]
			print 'State not recognized, setting it to Unkown'
		"""

	# Object Properties
	@property 
	def state(self):
		return self.properties.get('state', self.MAP[0])
	@state.setter
	def state(self, state):
		self.properties['state'] = state
	@state.deleter
	def state(self):
		del self.properties['state']

class ArDroneState(State, object):
	MAP = [
	'Unknown', 
	'Inited', 
	'Landed', 
	'Flying', 
	'Hovering', 
	'Test', 
	'Taking off', 
	'Flying', 
	'Landed', 
	'Looping']

class ControllerState(State, object):
	MAP = [
	'Unknown', 
	'On', 
	'Off'
	]

class Motor(BasicObject, object):
	"""docstring for Motors
	Simple object to read pwm of each motor. 
	"""
	def __init__(self, **kwargs):

		self.properties = dict( pwm = kwargs.get('pwm', 0) )

	def __str__(self):

		return str(self.pwm) 

	def set_pwm(self, pwm):

		self.pwm = pwm
	
	# Object Properties	
	@property 
	def pwm(self):
		return self.properties.get('pwm', 0.0)
	@pwm.setter
	def pwm(self, pwm):
		self.properties['pwm'] = pwm

	@pwm.deleter
	def pwm(self):
		del self.properties['pwm']
	
def iterate_test():
	obj = SixDofObject()
	obj.x = 1; 
	obj.y = -4;
	obj.z = 2;

	obj.pitch = pi
	obj.roll = -pi/5
	obj.yaw = 60;

	for i in obj:
		print i 

	for i in obj:
		print i

	print len(obj)
	
def state_test():
	print "State Test"
	state = State(state = 'Landed')
	print state

	state.set_state(4)
	print state

	state.set_state(-3)
	print state

	state.set_state('Flying')
	print state
	print state is 'Flying'
	print state == 'Flying'
	print id(state)


	state.set_state('Flyiiing')
	print state

	print state.state
	print state == 'Unknown' 
	print state == 0

def ardrone_state_test():
	print "ArDroneState Test"
	state = ArDroneState(state = 'Landed')
	print state

	state.set_state(4)
	print state

	state.set_state(-3)
	print state

	state.set_state('Flying')
	print state
	print state is 'Flying'
	print state == 'Flying'
	print id(state)


	state.set_state('Flyiiing')
	print state

	print state.state
	print state == 'Unknown' 
	print state == 0

def controller_state_test():
	print "Controller State Test"
	state = ControllerState(state = 'Off') 
	print state.state
	print state.MAP
	state.set_state('Go-to-Goal')
	print state.state

	print state.state is 'Go-to-Goal'



def motor_test():
	print "Motor Test"
	motor = Motor()
	print motor
	print motor.pwm
	motor.pwm = 25
	print motor
	print motor.pwm

	print vars(motor)
	
def sixdofobject_test():
	print "SixDofObject Test"
	position = SixDofObject()
	position.x = 2.0
	position.yaw = pi/4
	position.set_properties(dict(y = 4.0, pitch = -30))
	print position
	print position.x	
	print position.yaw
	print position.y 

def quaternion_test():
	quat = Quaternion()
	print quat.w
	print vars(quat)
	# help(quat)
	print quat
	print repr(quat)

def main():
	print 'Ok'
	ob = BasicObject()
	print ob.properties

	#iterate_test()
	#state_test()
	#motor_test()
	#sixdofobject_test()
	#quaternion_test()
	#ardrone_state_test()
	controller_state_test()
if __name__ == '__main__': main()