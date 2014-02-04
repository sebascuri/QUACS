from BasicObject import BasicObject
from collections import deque

class BasicController(BasicObject, object):
	"""docstring for Controller"""
	def __init__(self, **kwargs):
		super(BasicController, self).__init__( **kwargs )

		self.input = dict( position = kwargs.get( 'input', deque([0], maxlen = kwargs.get('input_size', kwargs.get('error_size', 1) ) ) ), 
			velocity = kwargs.get( 'input', deque([0], maxlen = kwargs.get('input_size', kwargs.get('error_size', 1) ) ) ) )

		self.output = kwargs.get( 'output', deque([0], maxlen = kwargs.get('output_size', 1) ) )

		self.error = dict( position = kwargs.get( 'input', deque([0], maxlen = kwargs.get('input_size', kwargs.get('error_size', 1) ) ) ), 
			velocity = kwargs.get( 'input', deque([0], maxlen = kwargs.get('input_size', kwargs.get('error_size', 1) ) ) ) )

		self.set_point = dict( position = kwargs.get('set_point', kwargs.get('position_set_point', deque([0],  maxlen = kwargs.get('set_point_size', 1) ) ) ), 
			velocity =  kwargs.get('derivative_set_point', kwargs.get('velocity_set_point', deque([0],  maxlen = kwargs.get('derivative_set_point_size', 1) ) ) ) )

		self.Ts = kwargs.get( 'Ts', kwargs.get('Command_Time', 0) )
	
	def change_set_point(self, *args, **kwargs):

		self.set_dict_deque(self.set_point, *args, **kwargs)

	def set_input(self, *args, **kwargs):
		self.set_dict_deque(self.input, *args, **kwargs)

		self.set_error()

	def set_dict_deque(self, attribute, *args, **kwargs):
		if len(args) == 1:
			arg = args[0]
			if type(arg) == dict():
				self.set_dict_deque(**arg)
			else:
				attribute['position'].append( arg ) 
				attribute['velocity'].append( 0 )

		elif len(args) == 2:
			attribute['position'].append(args[0])
			attribute['velocity'].append(args[1])

		for key, value in kwargs.items():
			attribute[key].append(value)

	def set_error(self):
		for key in self.input.keys( ):
			self.error[key].append( self.set_point[key][-1] - self.input[key][-1] )

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
	def error(self):
		return self.properties.get('error', None)
	@error.setter
	def error(self, new_error):
		if self.error is None:
			self.properties['error'] = new_error #first error
		else:
			self.error.append(new_error)
	@error.deleter
	def error(self):
		del self.properties['error']

	@property
	def set_point(self):
		return self.properties.get('set_point', None)
	@set_point.setter
	def set_point(self, new_set_point):
		if self.set_point is None:
			self.properties['set_point'] = new_set_point #first set_point
		else:
			self.change_set_point(new_set_point)
	@set_point.deleter
	def set_point(self):
		del self.properties['set_point']

	@property
	def Ts(self):
		return self.properties.get('Ts', 0.0)
	@Ts.setter
	def Ts(self, Ts):
		self.properties['Ts'] = Ts
	@Ts.deleter
	def Ts(self):
		del self.properties['Ts']


class PID_Controller(BasicController, object):
	"""docstring for Controller"""
	def __init__(self, **kwargs):
		super(PID_Controller, self).__init__( **kwargs )

		self.parallel_errors = dict(proportional = kwargs.get('p_error', 0), integral = kwargs.get('i_error', 0), derivative = kwargs.get('d_error', 0) )
		self.gains = dict(proportional = kwargs.get('Kp', 0), integral = kwargs.get('Ki', 0), derivative = kwargs.get('Kd', 0) )
		self.saturation = False 

	def set_input(self, *args, **kwargs):
		super(PID_Controller, self).set_input(*args, **kwargs)
		
		self.set_parallel_errors()
		self.Control()

	def set_parallel_errors(self, *args):
		if len(args) == 0:
			self.update_errors(self.error)
		else:
			self.update_errors(args[0])

	def update_errors(self, error_dict):
		if not self.saturation:
			self.parallel_errors['integral'] += self.Ts * error_dict['position'][-1]

		self.parallel_errors['proportional'] = error_dict['position'][-1]
		self.parallel_errors['derivative'] = error_dict['velocity'][-1]

	def Control(self):
		aux = 0 
		for key in self.parallel_errors.keys():
			aux += self.parallel_errors[key] * self.gains[key]

		self.output.append(aux)

	def Reset(self):
		""" Reset Errors to Cero"""
		for key in self.parallel_errors.keys():
			self.parallel_errors[key] = 0 

	# Object Properties
	@property
	def parallel_errors(self):
		return self.properties.get('parallel_errors', None)
	@parallel_errors.setter
	def parallel_errors(self, new_error):
		if self.parallel_errors is None:
			self.properties['parallel_errors'] = new_error
		else:
			self.set_parallel_errors(new_error)
	@parallel_errors.deleter
	def parallel_errors(self):
		del self.properties['parallel_errors'] 

	@property
	def gains(self):
		return self.properties.get('gains', None)
	@gains.setter
	def gains(self, new_gains):
		self.properties['gains'] = new_gains
	@gains.deleter
	def gains(self):
		del self.properties['gains'] 

	@property
	def saturation(self):
		return self.properties.get('saturation', None)
	@saturation.setter
	def saturation(self, new_saturation):
		self.properties['saturation'] = new_saturation
	@saturation.deleter
	def saturation(self):
		del self.properties['saturation'] 


		
def basicController( ):
	control = BasicController( )
	print control.input
	control.input = dict(yaw_error = 2)
	print control.input 
	control.new_input( yaw_error = 4)
	print control.input

	print control.output

def PIDTest( ):
	controller = PID_Controller( Ts = 1, Kp = 0, Ki = 1, Kd = 0 )
	controller.set_input(2.)

	print controller.input
	print controller.get_output()
	controller.set_input(1.5)

	print controller.input 
	print controller.get_output() 
	print controller.error 
	controller.saturation = True
	controller.set_input(2.)
	controller.set_input(2.)
	controller.set_input(2.)
	controller.set_input(2.)

	print controller.get_output()

def main():
	# basicController()
	PIDTest()

	
if __name__ == '__main__': main()