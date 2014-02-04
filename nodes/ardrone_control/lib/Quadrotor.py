#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi
from BasicObject import BasicObject, SixDofObject, Quaternion, Motor, ArDroneState
import Process
import Sensors
# from EKF import EKF

class Quadrotor(BasicObject, object):
	"""docstring for Quadrotor"""
	def __init__(self, **kwargs):
		self.properties = dict(
			position = kwargs.get('position', SixDofObject() ),
			orientation = kwargs.get('orientation', Quaternion() ) ,
			velocity = kwargs.get('velocity', SixDofObject() ),
			acceleration = kwargs.get('acceleration', SixDofObject() ),
			battery = kwargs.get('battery', 100 ) ,
			state = kwargs.get('state', ArDroneState()) ,
			motors = kwargs.get('motors', [Motor(), Motor(), Motor(), Motor()] )  ,
			processes = kwargs.get('processes', list() ),
			sensors = kwargs.get('sensors', list() )
			)

		

		"""
		self.position = kwargs.get('position', SixDofObject() )
		self.orientation = kwargs.get('orientation', Quaternion() ) 
		self.velocity = kwargs.get('velocity', SixDofObject() )
		self.acceleration = kwargs.get('acceleration', SixDofObject() )
		self.battery = kwargs.get('battery', 100 )
		self.state = kwargs.get('state', ArDroneState())
		self.motors = kwargs.get('motors', [Motor(), Motor(), Motor(), Motor()] ) 
		self.processes = kwargs.get('processes', list() )
		self.sensors = kwargs.get('sensors', list() )
		"""
		"""
		self.properties['position'] = SixDofObject()
		self.properties['orientation'] = Quaternion()
		self.properties['velocity'] = SixDofObject()
		self.properties['acceleration'] = SixDofObject()
		self.properties['battery'] = 100
		self.properties['motors'] = 
		self.properties['state'] = ArDroneState()
		self.properties['processes'] = kwargs.get('processes', list() )
		self.properties['sensors'] = kwargs.get('sensors', list() )
		"""

		# self.properties['filters'] = [EKF(processes = self.processes, sensors = self.sensors)]


	def predict( self ):
		for process in self.processes:
			changed = process.update( position = self.position, orientation = self.orientation, velocity = self.velocity, acceleration = self.acceleration )
			for attribute, value in changed.items():
				setattr(self, attribute, value)

	def set_attributes(self, X, stateList):
		i = 0
		for item in stateList:
			attribute, direction = item.split('.')
			setattr( getattr( self, attribute), direction, X.item(i) )
			i += 1

	def set_state(self, state):
		
		self.state.set_state(state)

	def set_pwm(self, *args):
		if len(args) == 1:
			i = 0
			for pwm in args[0]:
				self.motors[i].set_pwm( pwm )
				i += 1 
		elif len(args) == 4:
			i = 0
			for pwm in args:
				self.motors[i].set_pwm( pwm )
				i +=1 

	@property
	def position(self):
		return self.properties.get('position', SixDofObject())
	@position.setter
	def position(self, position):
		self.properties['position'] = position
	@position.deleter
	def position(self):
		del self.properties['position']	

	@property
	def orientation(self):
		return self.properties.get('orientation', Quaternion())
	@orientation.setter
	def orientation(self, orientation):
		self.properties['orientation'] = orientation
	@orientation.deleter
	def orientation(self):
		del self.properties['orientation']	

	@property
	def velocity(self):
		return self.properties.get('velocity', SixDofObject())
	@velocity.setter
	def velocity(self, velocity):
		self.properties['velocity'] = velocity
	@velocity.deleter
	def velocity(self):
		del self.properties['velocity']	

	@property
	def acceleration(self):
		return self.properties.get('acceleration', SixDofObject())
	@acceleration.setter
	def acceleration(self, acceleration):
		self.properties['acceleration'] = acceleration
	@acceleration.deleter
	def acceleration(self):
		del self.properties['acceleration']	


	@property
	def battery(self):
		return self.properties.get('battery', 100.0)
	@battery.setter
	def battery(self, battery):
		self.properties['battery'] = battery
	@battery.deleter
	def battery(self):
		del self.properties['battery']	

	@property
	def state(self):
		return self.properties.get('state', ArDroneState())
	@state.setter
	def state(self, state):
		self.set_state(state)
	@state.deleter
	def state(self):
		del self.properties['state']	

	@property
	def motors(self):
		return self.properties.get('motors', [Motor(), Motor(), Motor(), Motor()])
	@motors.setter
	def motors(self, motors):
		self.set_pwm(motors)
	@motors.deleter
	def motors(self):
		del self.properties['motors']	

	@property
	def processes(self):
		return self.properties.get('processes', list())
	@processes.setter
	def processes(self, processes):
		self.properties['processes'] = processes
	@processes.deleter
	def processes(self):
		del self.properties['processes']	

	@property
	def sensors(self):
		return self.properties.get('sensors', list())
	@sensors.setter
	def sensors(self, sensors):
		self.properties['sensors'] = sensors
	@sensors.deleter
	def sensors(self):
		del self.properties['sensors']

def quadrotor_test():
	print "Quadrotor Test"
	parrot = Quadrotor(battery = 80, position = SixDofObject(x = 2))
	print parrot.battery
	print parrot.position

	parrot.set_state('Flying')
	print parrot.state

	for motor in parrot.motors:
		print motor

	parrot.set_pwm([50, 70, 20, 40])
	for motor in parrot.motors:
		print motor
	
	print parrot.velocity
	parrot.velocity.x = 2
	print parrot.velocity
	parrot.velocity.set_attribute( x =  -3)
	print parrot.velocity
	parrot.velocity.set_attribute(dict(yaw = pi/2, pitch = 14, roll = -12))
	print parrot.velocity

	parrot.acceleration.set_attribute(dict(x = -3, y = 12))
	print parrot.acceleration 

	parrot.orientation.set_attribute(dict(x = 1, y = -2, w = 0.2))
	print parrot.orientation
	print parrot.orientation.w

	parrot.position.set_attribute( dict(z = -3, pitch = 67) )
	print parrot.position

	print parrot

	print "Attribute Test"

	parrot.position.set_attribute( dict( x = 10, y = 100, z = 1000, pitch = 10000, roll = 100000, yaw = 1000000) )
	for i in parrot.position:
		print i 

	parrot.velocity.set_attribute( dict( x = 20, y = 200, z = 2000, pitch = 20000, roll = 200000, yaw = 2000000) ) 
	for i in parrot.velocity:
		print i

	parrot.acceleration. set_attribute( dict( x = 30, y = 300, z = 3000, pitch = 30000, roll = 300000, yaw = 3000000) )
	for i in parrot.acceleration:
		print i

	parrot.orientation. set_attribute( dict( x = 40, y = 400, z = 4000, w = 40000) )
	for i in parrot.orientation:
		print i

	parrot.battery = 50
	print parrot.battery

	parrot.set_pwm([60, 61, 62, 63])
	for motor in parrot.motors:
		print motor

	parrot.set_state('Hovering')
	print parrot.state



def main():
	quadrotor_test()
	
	processes = [Process.XY_Odometry1(Ts = 1), Process.Z_Odometry2(Ts = 1) ]
	sensors = [ Sensors.GPS() ]

	parrot = Quadrotor(battery = 80, position = SixDofObject(x = 2, y = -2, yaw = pi), velocity = SixDofObject(x = 1, y = 0), processes = processes, sensors = sensors)
	parrot.predict()
	print parrot.position
	parrot.predict()
	parrot.predict()
	parrot.predict()
	print parrot.position

	#print parrot.velocity.yaw
	




if __name__ == '__main__': main()