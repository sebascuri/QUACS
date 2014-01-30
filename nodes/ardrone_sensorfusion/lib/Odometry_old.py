#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi, cos, sin
import numpy as np 

from Quadrotor import Quadrotor, BasicObject
class Odometry(BasicObject, object):
	"""docstring for Odometry"""
	def __init__(self, **kwargs):
		self.properties = dict()
		super(Odometry, self).__init__(**kwargs)

	def predict(self):
		self.robot.position.x += self.Ts  * (self.robot.velocity.x * cos(self.robot.position.yaw) - self.robot.velocity.y * sin(self.robot.position.yaw));
		self.robot.position.y += self.Ts  * (self.robot.velocity.x * sin(self.robot.position.yaw) + self.robot.velocity.y * cos(self.robot.position.yaw));

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
	def robot(self):
		return self.properties.get('robot', None)
	@robot.setter 
	def robot(self, robot):
		self.properties['robot'] = robot
	@robot.deleter
	def robot(self):
		del self.properties['robot']

	@property
	def Jacobian(self):
		self.properties['Jacobian'] = np.array( [ 
				[1, 0, self.Ts * ( - self.robot.velocity.x * sin(self.robot.position.yaw) - self.robot.velocity.y  * cos(self.robot.position.yaw))],
				[0, 1, self.Ts * ( + self.robot.velocity.x * cos(self.robot.position.yaw) + self.robot.velocity.y  * sin(self.robot.position.yaw))], 
				[0, 0, 1]])
		return self.properties.get('Jacobian')
	
def main():
	odometry = Odometry( robot = Quadrotor(), Ts = 1)
	print odometry
	print odometry.robot
	print odometry.Ts
	print odometry.Jacobian
	print odometry.properties
	
if __name__ == '__main__': main()