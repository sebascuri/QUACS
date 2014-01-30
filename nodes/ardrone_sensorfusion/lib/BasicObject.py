#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi
try:
	import tf
except ImportError:
	pass


class BasicObject(object):
	"""docstring for BasicObject"""
	def __init__(self, **kwargs):
		super(BasicObject, self).__init__()

		if not hasattr(self, 'properties'):
			self.properties = dict()
		
		for key, value in kwargs.items():
			self.properties[key] = value
	
	def __str__(self):

		return str(vars(self)) 
		
	def __repr__(self):

		return str( self.__class__.__name__) 

	def __len__(self):

		return len(vars(self)) 

	def __iter__(self):
		for key, value in vars(self).iteritems():
			yield key, value 

	def __eq__(self, data):

		return str(self) == data

	def get_properties(self):

		return self.properties;

	def get_property(self, key):

		return self.properties.get(key, None)

	def set_attribute(self, *args, **kwargs):
		if len(args) == 1:
			if type(args[0]) == dict:
				self.set_attribute(** args[0])

		for key, value in kwargs.items():
			setattr(self, key, value)

class SixDofObject(BasicObject, object):
	"""docstring for SixDofObject:
	This object has properties for each of the 6 degrees of freedom"""
	def __init__(self, **kwargs):
		self.x = kwargs.get('x', 0.0)
		self.y = kwargs.get('y', 0.0)
		self.z = kwargs.get('z', 0.0)
		self.yaw = kwargs.get('yaw', 0.0)
		self.pitch = kwargs.get('pitch', 0.0)
		self.roll = kwargs.get('roll', 0.0)

		# self.properties = dict(x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0)
		# super(SixDofObject, self).__init__(**kwargs)
	
	"""	
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
	"""

class Quaternion(BasicObject, object):
	def __init__(self, **kwargs):
		# self.properties = dict( x = 0.0, y = 0.0, z = 0.0, w = 1.0 )
		self.x = kwargs.get('x', 0.0)
		self.y = kwargs.get('y', 0.0)
		self.z = kwargs.get('z', 0.0)
		self.w = kwargs.get('w', 1.0)
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


	"""
	@property 
	def x(self):
		return self.properties.get('x', None)
	@x.setter 
	def x(self, x):
		self.properties['x'] = x
	@x.deleter
	def x(self):
		del self.properties['x']

	@property 
	def y(self):
		return self.properties.get('y', None)
	@y.setter 
	def y(self, y):
		self.properties['y'] = y
	@y.deleter
	def y(self):
		del self.properties['y']

	@property 
	def z(self):
		return self.properties.get('z', None)
	@z.setter 
	def z(self, z):
		self.properties['z'] = z
	@z.deleter
	def z(self):
		del self.properties['z']

	@property 
	def w(self):
		return self.properties.get('w', None)
	@w.setter 
	def w(self, w):
		self.properties['w'] = w
	@w.deleter
	def w(self):
		del self.properties['w']
	"""

class StateMapping(object):
	Unkown = 0
	Inited = 1
	Landed = 2
	Flying = 3
	Hovering = 4
	Test = 5
	TakingOff = 6
	Flying = 7
	Landed = 8
	Looping = 9
						
class State(BasicObject, object):
	"""docstring for State"""
	def __init__(self, state = 2):
		# super(State, self).__init__()
		self._STATES = {0 : 'Unknown', 1 : 'Inited', 2 : 'Landed', 3 : 'Flying', 4 : 'Hovering', 5 : 'Test', 6 : 'Taking off', 7 : 'Flying', 8 : 'Landed', 9 : 'Looping'}
		
		self.set_state(state)
		
	def __str__(self):
		return str(self.state)

	def __eq__(self, data):
		if type(data) == str:
			return str(self.state) is data
		elif type(data) == int:
			return str(self.state) is self._STATES[data]

	def set_state(self, state):
		if (state in self._STATES.values()) or (state in self._STATES.keys()):
			if type(state) == int:
				self.state = self._STATES[state]
			else:
				self.state = state
		else:
			self.state = self._STATES[0]
			print 'State not recognized, setting it to Unkown'

	"""
	@property 
	def state(self):
		return self.state 
	@state.setter
	def state(self, state):
		self.set_state(state)
	@state.deleter
	def state(self):
		del self.state
	"""
	
class Motor(BasicObject, object):
	"""docstring for Motors"""
	def __init__(self, **kwargs):
		self.pwm = kwargs.get('pwm', 0)

	def __str__(self):
		return str(self.pwm) 

	def set_pwm(self, pwm):
		self.pwm = pwm
	
	"""
	@property 
	def pwm(self):
		return self.properties.get('pwm', 0)
	@pwm.setter
	def pwm(self, pwm):
		self.properties['pwm'] = pwm

	@pwm.deleter
	def pwm(self):
		del self.properties['pwm']
	"""

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
	position.x = 2.0
	position.yaw = pi/4
	position.set_attribute(dict(y = 4.0, pitch = -30))
	print position
	print position.x	
	print position.yaw
	print position.y 

def quaternion_test():
	quat = Quaternion()
	print quat.w

def main():
	print 'Ok'
	ob = BasicObject()
	print ob.properties

	iterate_test()
	state_test()
	motor_test()
	sixdofobject_test()
	quaternion_test()



if __name__ == '__main__': main()