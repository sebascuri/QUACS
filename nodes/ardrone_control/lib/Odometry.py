#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi, cos, sin
import numpy as np 

from Quadrotor import Quadrotor

class Odometry(Quadrotor, object):
	"""docstring for Odometry"""
	def __init__(self, **kwargs):
		super(Odometry, self).__init__( **kwargs )

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
	def ProcessJacobian(self):
		return self.properties.get('ProcessJacobian')
	@ProcessJacobian.setter
	def ProcessJacobian(self, A):
		self.properties['ProcessJacobian'] = A
	@ProcessJacobian.deleter
	def ProcessJacobian(self):
		del self.properties['ProcessJacobian']	

class Odometry1(Odometry, object):
	"""docstring for Odometry
	First Order Odometry for AR.DRONE"""
	def __init__(self, **kwargs):
		super(Odometry, self).__init__( **kwargs )

	def update(self):
		self.position.x += self.Ts  * (self.velocity.x * cos(self.position.yaw) - self.velocity.y * sin(self.position.yaw));
		self.position.y += self.Ts  * (self.velocity.x * sin(self.position.yaw) + self.velocity.y * cos(self.position.yaw));

		self.ProcessJacobian = np.array( [ 
				[1, 0, self.Ts * ( - self.velocity.x * sin(self.position.yaw) - self.velocity.y  * cos(self.position.yaw))],
				[0, 1, self.Ts * ( + self.velocity.x * cos(self.position.yaw) + self.velocity.y  * sin(self.position.yaw))], 
				[0, 0, 1]])

class Odometry2(Odometry, object):
	"""docstring for Odometry2
	Second Order Odometry for AR.DRONE"""
	def __init__(self, **kwargs):
		super(Odometry2, self).__init__( **kwargs )

	def update(self):
		self.position.x += (self.Ts  * (self.velocity.x * cos(self.position.yaw) - self.velocity.y * sin(self.position.yaw)) 
			+ 0.5 * self.Ts ** 2 * (self.acceleration.x * cos(self.position.yaw) - self.velocity.x * self.velocity.yaw * sin(self.position.yaw) 
			- self.acceleration.y * sin(self.position.yaw) - self.velocity.y * self.velocity.yaw * cos(self.position.yaw) ) );
		
		self.position.y += (self.Ts  * (self.velocity.x * sin(self.position.yaw) + self.velocity.y * cos(self.position.yaw)) 
			+ 0.5 * self.Ts ** 2 * (self.acceleration.x * sin(self.position.yaw) + self.velocity.x * self.velocity.yaw * cos(self.position.yaw) 
			+ self.acceleration.y * cos(self.position.yaw) - self.velocity.y * self.velocity.yaw * sin(self.position.yaw) ) );

		self.ProcessJacobian = np.array( [ 
				[1, 0, self.Ts * ( - self.velocity.x * sin(self.position.yaw) - self.velocity.y  * cos(self.position.yaw)) + 0.5 * self.Ts ** 2 *( 
					- self.acceleration.x * sin(self.position.yaw) - self.velocity.x * self.velocity.yaw * cos(self.position.yaw) 
					- self.acceleration.y * cos(self.position.yaw) + self.velocity.y * self.velocity.yaw * sin(self.position.yaw) )],
				[0, 1, self.Ts * ( + self.velocity.x * cos(self.position.yaw) + self.velocity.y  * sin(self.position.yaw)) + 0.5 * self.Ts ** 2 * (
					+ self.acceleration.x * cos(self.position.yaw) - self.velocity.x * self.velocity.yaw * sin(self.position.yaw) 
					- self.acceleration.y * sin(self.position.yaw) - self.velocity.y * self.velocity.yaw * cos(self.position.yaw) ) ], 
				[0, 0, 1]])

		
	
def main():
	odometry = Odometry1( Ts = 1 )
	print odometry
	odometry.update()
	print odometry.Ts
	print odometry.ProcessJacobian
	#print odometry.properties

	odometry2 = Odometry2( Ts = 2)
	odometry2.update()

	print odometry2.Ts
	print odometry2.ProcessJacobian
	
if __name__ == '__main__': main()