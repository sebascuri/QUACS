#!/users/sebastiancuri/anaconda/bin/python
#!/usr/bin/env python

from math import pi, cos, sin, sqrt
import numpy as np 
import numpy.matlib as matlib 
import rospy;

try:
	import tf;
except ImportError:
	import roslib; roslib.load_manifest('ardrone_control')
	
	import tf;


# from Quadrotor import SixDofObject
from BasicObject import BasicObject, SixDofObject, Quaternion

def quaternion_matrix(quaternion):
	x = quaternion[0]; y = quaternion[1]; z = quaternion[2]; w = quaternion[3];
	return np.mat([ 
		[w, -x, -y, -z ],
		[x, w, z, -y ],
		[y, -z, w, x ],
		[z, y, -x, w]
		])

class BasicProcess(BasicObject, object):
	"""docstring for BasicProcess
	Each process has an predict mehtod that predict the variables in the state list 
	In each update variable X and ProcessJacobian is also updated
	"""
	def __init__(self, **kwargs):
		super(BasicProcess, self).__init__( **kwargs )
		self.properties.update( stateList = kwargs.get('stateList', self.stateList) )  #List from property to index in X state
		self.properties.update(
			X = kwargs.get('X', matlib.zeros( [len(self), 1] )),
			ProcessJacobian = kwargs.get('ProcessJacobian', matlib.eye( len(self) ) ),
			Covariance = kwargs.get('Covariance', 0.3 * matlib.eye( len(self) ) ),
			Ts = kwargs.get('Ts', 0.0),
			position = kwargs.get('position', SixDofObject()),
			quaternion = kwargs.get('quaternion', Quaternion()),
			velocity = kwargs.get('velocity', SixDofObject()),
			acceleration = kwargs.get('acceleration', SixDofObject()) 
			)

		self.time = rospy.get_time()
		"""
		self.stateList = kwargs.get('stateList', list())  #List from property to index in X state
		self.X = kwargs.get('X', matlib.zeros( [len(self.stateList), 1] ))
		self.ProcessJacobian = kwargs.get('ProcessJacobian', matlib.eye( len(self.stateList) ) )
		self.Covariance = kwargs.get('Covariance', 0.3 * matlib.eye( len(self) ))
		self.Ts = kwargs.get('Ts', None)


		self.position = kwargs.get('position', SixDofObject())
		self.quaternion = kwargs.get('quaternion', Quaternion())
		self.velocity = kwargs.get('velocity', SixDofObject())
		self.acceleration = kwargs.get('acceleration', SixDofObject())
		"""

	def __len__(self):
		return np.size(self.stateList) 

	def predict(self, **kwargs):
		for key, value in kwargs.items():
			setattr(self, key, value)

	def get_stateList(self):
		return self.stateList

	def set_X(self):
		i = 0
		for key in self.stateList:
			attribute, direction = key.split('.')
			self.X[ i ] = getattr(getattr(self, attribute), direction)
			i += 1

	def update_sampling_rate(self):
		aux = rospy.get_time()
		self.Ts = aux - self.time;
		self.time = aux;

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
		return self.properties.get('X', matlib.zeros( [len(self), 1] ) )
	@X.setter 
	def X(self, *args, **kwargs):
		for arg in args:
			if type(arg) == type( matlib.zeros(1) ):
				self.properties['X'] = arg 
			elif type(arg) == type( dict() ):
				for key, value in arg.items():
					self.X[ self.stateList.index(key) ][0] =  value

		for key, value in kwargs.items():
			self.X[ self.stateList.index(key) ][0] =  value

	@X.deleter
	def X(self):
		del self.properties['X']

	@property
	def ProcessJacobian(self):
		return self.properties.get('ProcessJacobian', matlib.eye( len(self) ) )
	@ProcessJacobian.setter
	def ProcessJacobian(self, A):
		self.properties['ProcessJacobian'] = A
	@ProcessJacobian.deleter
	def ProcessJacobian(self):
		del self.properties['ProcessJacobian']	

	@property
	def Covariance(self):
		return self.properties.get('Covariance', 0.3 * matlib.eye( len(self) ) )
	@Covariance.setter
	def Covariance(self, Q):
		self.properties['Covariance'] = Q 
	@Covariance.deleter
	def Covariance(self):
		del self.properties['Covariance']	


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
	def quaternion(self):
		return self.properties.get('quaternion', Quaternion())
	@quaternion.setter
	def quaternion(self, quaternion):
		self.properties['quaternion'] = quaternion
	@quaternion.deleter
	def quaternion(self):
		del self.properties['quaternion']	

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

class XY_Odometry1(BasicProcess, object):
	"""docstring for Odometry
	First Order Odometry for AR.DRONE
	x(k+1) = x(k) + vx(k).cos(yaw(k)).Ts - vy(k).sin(yaw(k)).Ts
	y(k+1) = y(k) + vx(k).sin(yaw(k)).Ts + vy(k).cos(yaw(k)).Ts
	"""
	stateList = [ 'position.x', 'position.y' , 'position.yaw' ]
	def __init__(self, *args, **kwargs):	
		super(XY_Odometry1, self).__init__(stateList = XY_Odometry1.stateList, *args, **kwargs )

	def predict(self, **kwargs):
		super(XY_Odometry1, self).predict( **kwargs )

		self.position.x += self.Ts * ( self.velocity.x * cos(self.position.yaw) - self.velocity.y * sin(self.position.yaw) )
		self.position.y += self.Ts * ( self.velocity.x * sin(self.position.yaw) + self.velocity.y * cos(self.position.yaw) )

		self.ProcessJacobian = np.matrix([ 
				[1, 0, self.Ts * ( - self.velocity.x * sin(self.position.yaw) - self.velocity.y  * cos(self.position.yaw))],
				[0, 1, self.Ts * ( + self.velocity.x * cos(self.position.yaw) + self.velocity.y  * sin(self.position.yaw))], 
				[0, 0, 1] ])

		self.set_X( )

		return dict(position = self.position)

class Z_Odometry1(BasicProcess, object):
	"""docstring for Z_Odometry1
	z(k+1) = z(k) + vz(k).Ts 
	"""
	stateList = ['position.z']
	def __init__(self, *args, **kwargs):
		stateList = ['position.z']
		super(Z_Odometry1, self).__init__(stateList = Z_Odometry1.stateList, *args, **kwargs )

	def predict(self, **kwargs):
		super(Z_Odometry1, self).predict( **kwargs )


		self.position.z += self.Ts * self.velocity.z

		self.ProcessJacobian = np.matrix( [[ 1 ]])

		self.set_X( )
		return dict(position = self.position)

class Odometry(BasicProcess, object):
	"""docstring for Odometry
	First Order Odometry for AR.DRONE
	x(k+1) = x(k) + vx(k).cos(yaw(k)).Ts - vy(k).sin(yaw(k)).Ts
	y(k+1) = y(k) + vx(k).sin(yaw(k)).Ts + vy(k).cos(yaw(k)).Ts
	z(k+1) = z(k) + vz(k).Ts 
	"""
	stateList = [ 'position.x', 'position.y' , 'position.z' , 'position.yaw' ]
	def __init__(self, *args, **kwargs):	
		super(Odometry, self).__init__(*args, **kwargs)

	def predict(self, **kwargs):

		#super(Odometry, self).predict( **kwargs )
		
		self.update_sampling_rate()


		self.position.x += self.Ts * ( self.velocity.x * cos(self.position.yaw) - self.velocity.y * sin(self.position.yaw) )
		self.position.y += self.Ts * ( self.velocity.x * sin(self.position.yaw) + self.velocity.y * cos(self.position.yaw) )
		self.position.z += self.Ts * self.velocity.z 

		self.ProcessJacobian[0,3] = self.Ts * ( - self.velocity.x * sin(self.position.yaw) - self.velocity.y  * cos(self.position.yaw))
		self.ProcessJacobian[1,3] = self.Ts * ( + self.velocity.x * cos(self.position.yaw) + self.velocity.y  * sin(self.position.yaw))
		
		self.set_X( )
		
		#return dict(position = self.position)

	def set_position(self):
		for key in self.stateList:
			direction = key.split('.')[1]
			if not direction == 'yaw':
				setattr(self.position, direction, self.X.item( self.stateList.index(key) ) )
		
class XY_Odometry2(BasicProcess, object):
	"""docstring for Odometry2
	Second Order Odometry for AR.DRONE
	"""
	stateList = [ 'position.x', 'position.y' , 'position.yaw' ]
	def __init__(self, *args, **kwargs):
		
		super(XY_Odometry2, self).__init__(stateList = XY_Odometry2.stateList, *args, **kwargs )

	def predict(self, **kwargs):
		super(XY_Odometry2, self).predict( **kwargs )

		self.position.x += (self.Ts  * (self.velocity.x * cos(self.position.yaw) - self.velocity.y * sin(self.position.yaw)) 
			+ 0.5 * self.Ts ** 2 * (self.acceleration.x * cos(self.position.yaw) - self.velocity.x * self.velocity.yaw * sin(self.position.yaw) 
			- self.acceleration.y * sin(self.position.yaw) - self.velocity.y * self.velocity.yaw * cos(self.position.yaw) ) );
		
		self.position.y += (self.Ts  * (self.velocity.x * sin(self.position.yaw) + self.velocity.y * cos(self.position.yaw)) 
			+ 0.5 * self.Ts ** 2 * (self.acceleration.x * sin(self.position.yaw) + self.velocity.x * self.velocity.yaw * cos(self.position.yaw) 
			+ self.acceleration.y * cos(self.position.yaw) - self.velocity.y * self.velocity.yaw * sin(self.position.yaw) ) );

		self.ProcessJacobian = np.matrix( [ 
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
	stateList = ['position.z']
	def __init__(self, *args, **kwargs):
		super(Z_Odometry2, self).__init__(stateList = Z_Odometry2.stateList, *args, **kwargs )

	def predict(self, **kwargs):
		super(Z_Odometry2, self).predict( **kwargs )

		self.position.z += self.Ts * self.velocity.z + 0.5 * self.Ts ** 2 * self.acceleration.z 
		
		self.ProcessJacobian = np.matrix( [[ 1 ]])

		self.set_X( )

		return dict(position = self.position)

class SO3(BasicProcess, object):
	"""docstring for SO3
	SO3 predicition in quaternion form. 
	"""
	stateList = ['quaternion.w', 'quaternion.x', 'quaternion.y', 'quaternion.z']
	def __init__(self, *args, **kwargs):	
		super(SO3, self).__init__(stateList = SO3.stateList, *args, **kwargs)
		self.X = np.mat([1.0, 0.0, 0.0, 0.0]).transpose()

	def predict(self, **kwargs):
		#super(SO3, self).predict( **kwargs )
		
		self.update_sampling_rate()


		qdot = 0.5 * np.roll( np.matrix([
				tf.transformations.quaternion_multiply( 
					( self.quaternion.get_quaternion() ), 
					( self.velocity.roll, self.velocity.pitch, self.velocity.yaw, 0 ), 
					)
			]), 1).transpose() 

		self.X += self.Ts * qdot

		self.set_quaternion()

		# self.ProcessJacobian = quaternion_matrix( self.quaternion.get_quaternion( ) )
		
		# return dict(quaternion = self.quaternion)	

	def set_quaternion(self):
		""" sets quaternion from state vector
			normalizes quaternion
			resets state vector normalized
		"""
		for key in self.stateList:
			attribute = key.split('.')[0]
			direction = key.split('.')[1]
			if 'quaternion' == attribute:
				setattr( self.quaternion, direction, self.X.item( self.stateList.index(key) ) )
		self.quaternion.normalize()

		for key in self.stateList:
			attribute = key.split('.')[0]
			direction = key.split('.')[1]
			if 'quaternion' == attribute:
				self.X[ self.stateList.index(key) ] = getattr(self.quaternion, direction)  

	def get_quaternion(self):
		return self.quaternion.properties

	def get_eulers(self):
		return self.quaternion.get_euler()

def process_test( process_object ):
	process = process_object;
	print repr(process)

	print 'X', process.X 
	process.predict(position = SixDofObject(x = 2, y = -1, z = 3, yaw = pi/4), velocity = SixDofObject(x = 3, y = 2, z = -2, pitch = pi/2, roll = -pi/6 ), orientation = Quaternion(w = 1, x = 0) , acceleration = SixDofObject(x = -1, y = 2, z = 5))
	print 'X', process.X
	print 'P', process.ProcessJacobian
	print 'R', process.Covariance
	process.predict(position = SixDofObject(x = 5, y = -1, z = 2), velocity = SixDofObject(x = 3, y = 2, z = -2, yaw = pi/4), orientation = Quaternion(w = 1, x = 0) , acceleration = SixDofObject(x = -1, y = 2, z = 0))

	print 'X', process.X
	process.predict()
	print 'X', process.X 
	process.predict()
	print 'X', process.X 

	print 'P', process.ProcessJacobian
	print 'R', process.Covariance

	print vars(process).keys()

def main():
	process_test( XY_Odometry1( Ts = 1 ) )
	#process_test( XY_Odometry2( Ts = 1 ) )
	#process_test( Z_Odometry1( Ts = 1 ) )
	#process_test( Z_Odometry2( Ts = 1 ) )
	#process_test( SO3(Ts = 1) )



	
if __name__ == '__main__': main()