#!/usr/bin/env python

from BasicObject import BasicObject
from collections import deque
import math

import numpy as np 
import numpy.matlib as matlib
import numpy.linalg as linalg
import scipy.linalg as sp 
import rospy

try:
	import tf;
except ImportError:
	pass



class DigitalFilter(BasicObject, object):
	"""docstring for DigitalFilter
		A digital filtered implemented as a Direct Form II parallel algorithm. 
		H(z) = Output / Input = b[i].z^-i / a[i].z^-i 
		ouptut [n] = - a[k] output[n-k] + b[k].input[n-k] 
	"""
	def __init__(self, **kwargs):
		super(DigitalFilter, self).__init__( **kwargs )

		self.b = kwargs.get('b', kwargs.get('numerator', [0] )  )
		self.a = kwargs.get('a', kwargs.get('denominator', [0] ) )

		self.input = deque( maxlen = kwargs.get('input_size', len(self.b) ) )

		self.output = deque( maxlen = kwargs.get('output_size', len(self.a)) )

		self.Ts = kwargs.get( 'Ts', kwargs.get('Command_Time', 0) )

	def set_input(self, *args, **kwargs):
		if len(args) == 1:
			arg = args[0]
			if type(arg) == dict():
				self.set_input( **arg )
			else:
				self.input.append(arg)
		
		for key, value in kwargs.items():
			self.input.append(value)

		self.Filter()

		return self.get_output()

	def Filter(self):
		try:
			new_output = 0 
			for i in range(len(self.b)):
				new_output += self.b[i] * self.input[-1 - i]
			for i in range(len(self.a)):
				new_output -= self.a[i] * self.output[-1 - i]

			self.output.append(new_output)

		except IndexError:
			self.output.append( self.input[-1] )

	def get_output(self):

		return self.output[-1]

	# Object Properties
	@property
	def input(self):
		return self.properties.get('input', None)
	@input.setter
	def input(self, new_input):
		if self.input is None:
			self.properties['input'] = new_input #first input
		else:
			self.set_input(new_input) #sets new input and then controls 
		# self.properties['input'] = new_input
	@input.deleter
	def input(self):
		del self.properties['input']

	@property
	def output(self):
		return self.properties.get('output', None)
	@output.setter
	def output(self, new_output):
		if self.output is None:
			self.properties['output'] = new_output #first output
		else:
			self.output.append(new_output)
	@output.deleter
	def output(self):
		del self.properties['output']

	@property
	def Ts(self):
		return self.properties.get('Ts', 0.0)
	@Ts.setter
	def Ts(self, Ts):
		self.properties['Ts'] = Ts
	@Ts.deleter
	def Ts(self):
		del self.properties['Ts']

	@property
	def a(self):
		return self.properties.get('a', 0.0)
	@a.setter
	def a(self, a):
		self.properties['a'] = a
	@a.deleter
	def a(self):
		del self.properties['a']

	@property
	def b(self):
		return self.properties.get('b', 0.0)
	@b.setter
	def b(self, b):
		self.properties['b'] = b
	@b.deleter
	def b(self):
		del self.properties['b']

class ExtendedKalmanFilter(BasicObject, object):
	"""docstring for ExtendedKalmanFilter"""
	def __init__(self, **kwargs):
		super(ExtendedKalmanFilter, self).__init__()

	def predict_error(self):
		self.ErrorCovariance = self.ProcessJacobian * self.ErrorCovariance * self.ProcessJacobian.transpose() + self.ProcessCovariance

	def correct(self):
		self.KalmanGain = self.ErrorCovariance * self.MeasurementJacobian.transpose() *  np.linalg.inv( self.MeasurementJacobian * self.ErrorCovariance * self.MeasurementJacobian.transpose() + self.MeasurementCovariance )
		self.X = self.X + self.KalmanGain * ( self.Z - self.StateMap)
		self.ErrorCovariance = (matlib.eye(np.size( self.X ) ) - self.KalmanGain * self.MeasurementJacobian ) * self.ErrorCovariance


	@property
	def X(self):
		return self.properties.get('X', None)
	@X.setter
	def X(self, X):
		self.properties['X'] = X
	@X.deleter
	def X(self):
		del self.properties['X']

	@property
	def Z(self):
		return self.properties.get('Z', None)
	@Z.setter
	def Z(self, Z):
		self.properties['Z'] = Z
	@Z.deleter
	def Z(self):
		del self.properties['Z']

	@property
	def StateMap(self):
		return self.properties.get('StateMap', None)
	@StateMap.setter
	def StateMap(self, StateMap):
		self.properties['StateMap'] = StateMap
	@StateMap.deleter
	def StateMap(self):
		del self.properties['StateMap']

	@property 
	def ProcessJacobian(self):
		return self.properties.get('ProcessJacobian', None)
	@ProcessJacobian.setter
	def ProcessJacobian(self, Q):
		self.properties['ProcessJacobian'] = Q
	@ProcessJacobian.deleter
	def ProcessJacobian(self):
		del self.properties['ProcessJacobian']

	@property 
	def MeasurementJacobian(self):
		return self.properties.get('MeasurementJacobian', None)
	@MeasurementJacobian.setter
	def MeasurementJacobian(self, Q):
		self.properties['MeasurementJacobian'] = Q
	@MeasurementJacobian.deleter
	def MeasurementJacobian(self):
		del self.properties['MeasurementJacobian']

	@property 
	def ErrorCovariance(self):
		return self.properties.get('ErrorCovariance', np.matlib.zeros([len(self), len(self)]) )
	@ErrorCovariance.setter
	def ErrorCovariance(self, P):
		self.properties['ErrorCovariance'] = P
	@ErrorCovariance.deleter
	def ErrorCovariance(self):
		del self.properties['ErrorCovariance']

	@property 
	def ProcessCovariance(self):
		return self.properties.get('ProcessCovariance', None)
	@ProcessCovariance.setter
	def ProcessCovariance(self, Q):
		self.properties['ProcessCovariance'] = Q
	@ProcessCovariance.deleter
	def ProcessCovariance(self):
		del self.properties['ProcessCovariance']

	@property 
	def MeasurementCovariance(self):
		return self.properties.get('MeasurementCovariance', None)
	@MeasurementCovariance.setter
	def MeasurementCovariance(self, R):
		self.properties['MeasurementCovariance'] = R 
	@MeasurementCovariance.deleter
	def MeasurementCovariance(self):
		del self.properties['MeasurementCovariance']

	@property
	def KalmanGain(self):
		return self.properties.get('KalmanGain', np.matlib.zeros([len(self), len(self)]))
	@KalmanGain.setter
	def KalmanGain(self, K):
		self.properties['KalmanGain'] = K 
	@KalmanGain.deleter
	def KalmanGain(self):
		del self.properties['KalmanGain']

class KalmanFilter(ExtendedKalmanFilter, object):
	def __init__(self, **kwargs):
		super(KalmanFilter, self).__init__()

	def predict(self):
		self.X += self.Ts * (self.ProcessJacobian * self.X )
		self.predict_error()

	def correct(self):
		self.StateMap = self.MeasurementJacobian * self.X 
		super(self).predict()

class MagdwickFilter(BasicObject, object):
	def __init__(self, **kwargs):
		super(MagdwickFilter, self).__init__()
		self.Beta = rospy.get_param('Magdwick/Beta', kwargs.get('Beta', 0.1) )

 	def correct(self):
 		F = np.mat(tf.transformations.quaternion_matrix( self.quaternion.get_quaternion() ).transpose()[0:3, 2] - self.sensors['accelerometer'].get_measurementvector()).transpose()

		J = np.mat( [
			[ -2*self.quaternion.y,	2*self.quaternion.z,    -2*self.quaternion.w,	2*self.quaternion.x ], 
			[ 2*self.quaternion.x,     2*self.quaternion.w,     2*self.quaternion.z,	2*self.quaternion.y], 
			[ 0,         -4*self.quaternion.x,    -4*self.quaternion.y,	0    ], 
			]).transpose()


		step = (np.dot(J, F))
		step = step / linalg.norm(step)

		self.X -= self.Ts*self.Beta * step
		self.set_quaternion()

	@property
	def stateList(self):
		return self.properties.get('stateList', list())
	@stateList.setter
	def stateList(self, new_list):
		self.properties['stateList'] = new_list
	@stateList.deleter
	def stateList(self):
		del self.properties['stateList']

	@property
	def X(self):
		return self.properties.get('X', None)
	@X.setter
	def X(self, X):
		self.properties['X'] = X
	@X.deleter
	def X(self):
		del self.properties['X']

	@property
	def Ts(self):
		return self.properties.get('Ts', 0.01)
	@Ts.setter
	def Ts(self, Ts):
		self.properties['Ts'] = Ts
	@Ts.deleter
	def Ts(self):
		del self.properties['Ts']

	@property
	def Beta(self):
		return self.properties.get('Beta', 0.1)
	@Beta.setter
	def Beta(self, Beta):
		self.properties['Beta'] = Beta
	@Beta.deleter
	def Beta(self):
		del self.properties['Beta']

class MahoneyFilter(BasicObject, object):
	def __init__(self, **kwargs):
		super(MahoneyFilter, self).__init__()

		self.Kp = rospy.get_param('Mahoney/Kp', kwargs.get('Kp', 0.1) )
		self.Ki = rospy.get_param('Mahoney/Ki', kwargs.get('Ki', 0.1) )

		self.error = dict(
			proportional = np.array([0., 0., 0.]).transpose(), 
			integral = np.array([0., 0., 0.]).transpose() )


	def correct(self):
		v = tf.transformations.quaternion_matrix( self.quaternion.get_quaternion() ).transpose()[0:3, 2]
		
		self.error['proportional'] = np.cross( self.sensors['accelerometer'].get_measurementvector(), v)

		self.error['integral'] += self.Ts * self.error['proportional']  

		u = self.Kp * self.error['proportional']   + self.Ki * self.error['integral'];  

		qdot = 0.5 * np.matrix([
			np.roll( 
				tf.transformations.quaternion_multiply( 
					( self.quaternion.get_quaternion() ), 
					(u[0], u[1], u[2], 0 ), 
					), 1)
			]).transpose()

		self.X += self.Ts * qdot

		self.set_quaternion()



	@property
	def stateList(self):
		return self.properties.get('stateList', list())
	@stateList.setter
	def stateList(self, new_list):
		self.properties['stateList'] = new_list
	@stateList.deleter
	def stateList(self):
		del self.properties['stateList']

	@property
	def X(self):
		return self.properties.get('X', None)
	@X.setter
	def X(self, X):
		self.properties['X'] = X
	@X.deleter
	def X(self):
		del self.properties['X']

	@property
	def Kp(self):
		return self.properties.get('Kp', 0.1)
	@Kp.setter
	def Kp(self, Kp):
		self.properties['Kp'] = Kp
	@Kp.deleter
	def Kp(self):
		del self.properties['Kp']

	@property
	def Ki(self):
		return self.properties.get('Ki', 0.1)
	@Ki.setter
	def Ki(self, Ki):
		self.properties['Ki'] = Ki
	@Ki.deleter
	def Ki(self):
		del self.properties['Ki']

	@property
	def Ki(self):
		return self.properties.get('Ki', 0.1)
	@Ki.setter
	def Ki(self, Ki):
		self.properties['Ki'] = Ki
	@Ki.deleter
	def Ki(self):
		del self.properties['Ki']

	@property
	def error(self):
		return self.properties.get('error', dict( ))
	@error.setter
	def error(self, error):
		self.properties['error'] = error
	@error.deleter
	def error(self):
		del self.properties['error']

def main():
	import h5py
	with h5py.File('y_raw.h5', 'r') as f:
		input_list = f['/raw'].value 

	with h5py.File('y_filter.h5', 'r') as f:
		output_list = f['/filter'].value 

	F = DigitalFilter( a = [-2.369513007182038, 2.313988414415879, -1.054665405878567, 0.187379492368185], b = [0.004824343357716, 0.019297373430865, 0.028946060146297, 0.019297373430865, 0.004824343357716])

	# input_list = [1.089321004453552, 1.089321004453552, 0.910632468746812, 0.910632468746812, 1.089315556872495, 1.089315556872495, 0.910627021163934, 0.910627021163934, 
	# 1.089312999843352, 1.089312999843352, 0.910624464136612, 0.910624464136612, 1.089317446848816, 1.089317446848816, 0.910628911142076, 0.910628911142076, 1.089300548227687, 
	# 1.089300548227687, 0.910612012519126, 0.910612012519126, 1.089294211244080, 1.089294211244080, 0.910605675535519]

	i = 0
	for data in input_list:
		print data, F.set_input(data), output_list[i] 
		assert data == F.input[-1]
		if i > 50: 
			assert math.fabs(output_list[i] - F.get_output()) < 10**-4


		i+=1


if __name__ == '__main__': main()