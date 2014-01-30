#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi, cos, sin
import numpy as np 

from Quadrotor import Quadrotor, BasicObject
class Odometry(Quadrotor, object):
	"""docstring for Odometry"""
	def __init__(self, **kwargs):
		self.properties = dict()
		super(Odometry, self).__init__( **kwargs )

	def predict(self):
		self.position.x += self.Ts  * (self.velocity.x * cos(self.position.yaw) - self.velocity.y * sin(self.position.yaw));
		self.position.y += self.Ts  * (self.velocity.x * sin(self.position.yaw) + self.velocity.y * cos(self.position.yaw));

	@property 
	def Ts(self):
		return self.properties.get('Ts', None)
	@Ts.setter 
	def Ts(self, Ts):
		self.properties['Ts'] = Ts
	@Ts.deleter
	def Ts(self):
		del self.properties['Ts']

	@property
	def Jacobian(self):
		self.properties['Jacobian'] = np.array( [ 
				[1, 0, self.Ts * ( - self.velocity.x * sin(self.position.yaw) - self.velocity.y  * cos(self.position.yaw))],
				[0, 1, self.Ts * ( + self.velocity.x * cos(self.position.yaw) + self.velocity.y  * sin(self.position.yaw))], 
				[0, 0, 1]])
		return self.properties.get('Jacobian')
	
def main():
	odometry = Odometry( Ts = 1)
	print odometry
	print odometry.Ts
	print odometry.Jacobian
	print odometry.properties
	
if __name__ == '__main__': main()