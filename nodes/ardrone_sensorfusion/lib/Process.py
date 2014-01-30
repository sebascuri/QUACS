#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi, cos, sin, sqrt
import numpy as np 
import numpy.matlib as matlib 

# from Quadrotor import SixDofObject
from BasicObject import BasicObject, SixDofObject, Quaternion

class BasicProcess(BasicObject, object):
	"""docstring for Odometry"""
	def __init__(self, **kwargs):
		# super(BasicProcess, self).__init__( **kwargs )
		self._stateList = kwargs.get('stateList', list())  #List from property to index in X state
		self.X = kwargs.get('X', matlib.zeros( [len(self._stateList), 1] ))
		self.Jacobian = kwargs.get('Jacobian', matlib.eye( len(self._stateList) ) )
		self.Covariance = kwargs.get('Covariance', 0.3 * matlib.eye( len(self) ))
		self.Ts = kwargs.get('Ts', None)


		self.position = kwargs.get('position', SixDofObject())
		self.orientation = kwargs.get('orientation', Quaternion())
		self.velocity = kwargs.get('velocity', SixDofObject())
		self.acceleration = kwargs.get('acceleration', SixDofObject())

	def __len__(self):
		return np.size(self._stateList) 

	def update(self, **kwargs):
		for key, value in kwargs.items():
			setattr(self, key, value)

	def get_stateList(self):
		return self._stateList

	def set_X(self):
		i = 0
		for key in self._stateList:
			attribute, direction = key.split('.')
			self.X[[i]] = getattr(getattr(self, attribute), direction)
			i += 1
	"""
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
	def X(self):
		return self.properties.get('X', None)
	@X.setter 
	def X(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( matlib.zeros(1) ):
				self.properties['X'] = arg 
			elif type(arg) == type( dict() ):
				for key, value in arg.items():
					self.X[ self._stateList.index(key) ][0] =  value

		for key, value in kwargs.items():
			self.X[ self._stateList.index(key) ][0] =  value

	@X.deleter
	def X(self):
		del self.properties['X']

	@property
	def Jacobian(self):
		return self.properties.get('Jacobian', None)
	@Jacobian.setter
	def Jacobian(self, A):
		self.properties['Jacobian'] = A
	@Jacobian.deleter
	def Jacobian(self):
		del self.properties['Jacobian']	

	@property
	def Covariance(self):
		return self.properties.get('Covariance', None)
	@Covariance.setter
	def Covariance(self, Q):
		self.properties['Covariance'] = Q 
	@Covariance.deleter
	def Covariance(self):
		del self.properties['Covariance']	

	"""

class XY_Odometry1(BasicProcess, object):
	"""docstring for Odometry
	First Order Odometry for AR.DRONE"""
	def __init__(self, *args, **kwargs):
		self._stateList = [ 'position.x', 'position.y' , 'position.yaw' ]
		super(XY_Odometry1, self).__init__(stateList = self._stateList, *args, **kwargs )

	def update(self, **kwargs):
		super(XY_Odometry1, self).update( **kwargs )

		self.position.x += self.Ts * ( self.velocity.x * cos(self.position.yaw) - self.velocity.y * sin(self.position.yaw) )
		self.position.y += self.Ts * ( self.velocity.x * sin(self.position.yaw) + self.velocity.y * cos(self.position.yaw) )

		self.Jacobian = np.matrix([ 
				[1, 0, self.Ts * ( - self.velocity.x * sin(self.position.yaw) - self.velocity.y  * cos(self.position.yaw))],
				[0, 1, self.Ts * ( + self.velocity.x * cos(self.position.yaw) + self.velocity.y  * sin(self.position.yaw))], 
				[0, 0, 1] ])

		self.set_X( )

		return dict(position = self.position)

class Z_Odometry1(BasicProcess, object):
	"""docstring for Z_Odometry1"""
	def __init__(self, *args, **kwargs):
		self._stateList = ['position.z']
		super(Z_Odometry1, self).__init__(stateList = self._stateList, *args, **kwargs )

	def update(self, **kwargs):
		super(Z_Odometry1, self).update( **kwargs )


		self.position.z += self.Ts * self.velocity.z

		self.Jacobian = np.matrix( [[ 1 ]])

		self.set_X( )
		return dict(position = self.position)
		
class XY_Odometry2(BasicProcess, object):
	"""docstring for Odometry2
	Second Order Odometry for AR.DRONE"""
	def __init__(self, *args, **kwargs):
		self._stateList = [ 'position.x', 'position.y' , 'position.yaw' ]
		super(XY_Odometry2, self).__init__(stateList = self._stateList, *args, **kwargs )

	def update(self, **kwargs):
		super(XY_Odometry2, self).update( **kwargs )

		self.position.x += (self.Ts  * (self.velocity.x * cos(self.position.yaw) - self.velocity.y * sin(self.position.yaw)) 
			+ 0.5 * self.Ts ** 2 * (self.acceleration.x * cos(self.position.yaw) - self.velocity.x * self.velocity.yaw * sin(self.position.yaw) 
			- self.acceleration.y * sin(self.position.yaw) - self.velocity.y * self.velocity.yaw * cos(self.position.yaw) ) );
		
		self.position.y += (self.Ts  * (self.velocity.x * sin(self.position.yaw) + self.velocity.y * cos(self.position.yaw)) 
			+ 0.5 * self.Ts ** 2 * (self.acceleration.x * sin(self.position.yaw) + self.velocity.x * self.velocity.yaw * cos(self.position.yaw) 
			+ self.acceleration.y * cos(self.position.yaw) - self.velocity.y * self.velocity.yaw * sin(self.position.yaw) ) );

		self.Jacobian = np.matrix( [ 
				[1, 0, self.Ts * ( - self.velocity.x * sin(self.position.yaw) - self.velocity.y  * cos(self.position.yaw)) + 0.5 * self.Ts ** 2 *( 
					- self.acceleration.x * sin(self.position.yaw) - self.velocity.x * self.velocity.yaw * cos(self.position.yaw) 
					- self.acceleration.y * cos(self.position.yaw) + self.velocity.y * self.velocity.yaw * sin(self.position.yaw) )],
				[0, 1, self.Ts * ( + self.velocity.x * cos(self.position.yaw) + self.velocity.y  * sin(self.position.yaw)) + 0.5 * self.Ts ** 2 * (
					+ self.acceleration.x * cos(self.position.yaw) - self.velocity.x * self.velocity.yaw * sin(self.position.yaw) 
					- self.acceleration.y * sin(self.position.yaw) - self.velocity.y * self.velocity.yaw * cos(self.position.yaw) ) ], 
				[0, 0, 1]])


		self.set_X( )

		return dict(position = self.position)

class Z_Odometry2(BasicProcess, object):
	"""docstring for Z_Odometry1"""	
	def __init__(self, *args, **kwargs):
		self._stateList = {'position.z' : 0}
		super(Z_Odometry2, self).__init__(stateList = self._stateList, *args, **kwargs )

	def update(self, **kwargs):
		super(Z_Odometry2, self).update( **kwargs )

		self.position.z += self.Ts * self.velocity.z + 0.5 * self.Ts ** 2 * self.acceleration.z 
		
		self.Jacobian = np.matrix( [[ 1 ]])

		self.set_X( )

		return dict(position = self.position)

class SO3(BasicProcess, object):
	"""docstring for SO3"""
	def __init__(self, *args, **kwargs):
		self._stateList = ['orientation.w', 'orientation.x', 'orientation.y', 'orientation.z']
		super(SO3, self).__init__(stateList = self._stateList, *args, **kwargs)
		
	def update(self, **kwargs):
		super(SO3, self).update( **kwargs )
		
		self.orientation.w += 0.5 * self.Ts * ( - self.velocity.roll * self.orientation.x - 
			self.velocity.pitch * self.orientation.y - self.velocity.yaw * self.orientation.z ) 

		self.orientation.x += 0.5 * self.Ts * ( + self.velocity.roll * self.orientation.w + 
			self.velocity.yaw * self.orientation.y - self.velocity.pitch * self.orientation.z )

		self.orientation.y += 0.5 * self.Ts * ( + self.velocity.pitch * self.orientation.w - 
			self.velocity.yaw * self.orientation.y - self.velocity.roll * self.orientation.z)

		self.orientation.z += 0.5 * self.Ts * ( + self.velocity.yaw * self.orientation.w + 
			self.velocity.pitch * self.orientation.x - self.velocity.roll * self.orientation.y)

		norm = 0
		for key, value in self.orientation:
			norm += value ** 2

		norm = sqrt(norm)

		for key, value in self.orientation:
			setattr(self.orientation, key , value/norm)


		self.Jacobian = np.matrix( [ 
			[ 1, - 0.5 * self.Ts *  self.velocity.roll, - 0.5 * self.Ts * self.velocity.pitch, - 0.5 * self.Ts * self.velocity.yaw ], 
			[ 0.5 * self.Ts * self.velocity.roll, 1, 0.5 * self.Ts * self.velocity.yaw, - 0.5 * self.Ts * self.velocity.pitch ],
			[ 0.5 * self.Ts * self.velocity.pitch, - 0.5 * self.Ts * self.velocity.yaw, 1, 0.5 * self.Ts * self.velocity.roll], 
			[ 0.5 * self.Ts * self.velocity.yaw, 0.5 * self.Ts * self.velocity.pitch, - 0.5 * self.Ts * self.velocity.roll, 1] ] )

		
		self.set_X( )	

		return dict(orientation = self.orientation)	

def process_test( process_object ):
	process = process_object;
	print repr(process)

	print 'X', process.X 
	process.update(position = SixDofObject(x = 2, y = -1, z = 3, yaw = pi/4), velocity = SixDofObject(x = 3, y = 2, z = -2, pitch = pi/2, roll = -pi/6 ), orientation = Quaternion(w = 1, x = 0) , acceleration = SixDofObject(x = -1, y = 2, z = 5))
	print 'X', process.X
	print 'P', process.Jacobian
	print 'R', process.Covariance
	process.update(position = SixDofObject(x = 5, y = -1, z = 2), velocity = SixDofObject(x = 3, y = 2, z = -2, yaw = pi/4), orientation = Quaternion(w = 1, x = 0) , acceleration = SixDofObject(x = -1, y = 2, z = 0))

	print 'X', process.X
	process.update()
	print 'X', process.X 
	process.update()
	print 'X', process.X 

	print 'P', process.Jacobian
	print 'R', process.Covariance

def main():
	process_test( XY_Odometry1( Ts = 1 ) )
	process_test( XY_Odometry2( Ts = 1 ) )
	process_test( Z_Odometry1( Ts = 1 ) )
	process_test( Z_Odometry2( Ts = 1 ) )
	process_test( SO3(Ts = 1) )



	
if __name__ == '__main__': main()