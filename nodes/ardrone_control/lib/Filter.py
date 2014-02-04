#!/usr/bin/env python

from BasicObject import BasicObject
from collections import deque
import math

class DigitalFilter(BasicObject, object):
	"""docstring for Controller"""
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