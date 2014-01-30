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

class PID_Controller(BasicController, object):
	"""docstring for Controller"""
	def __init__(self, **kwargs):
		super(PID_Controller, self).__init__( **kwargs )

		self.parallel_errors = dict(proportional = kwargs.get('p_error', 0), integral = kwargs.get('i_error', 0), derivative = kwargs.get('d_error', 0) )
		self.gains = dict(proportional = kwargs.get('Kp', 0), integral = kwargs.get('Ki', 0), derivative = kwargs.get('Kd', 0) )
		self.saturation = False 

	def set_input(self, *args, **kwargs):
		super(PID_Controller, self).set_input(*args, **kwargs)
		
		if not self.saturation:
			self.parallel_errors['integral'] += self.Ts * self.error['position'][-1]

		self.parallel_errors['proportional']  = self.error['position'][-1]
		self.parallel_errors['derivative'] = self.error['velocity'][-1]

		self.Control()

	def Control(self):
		aux = 0 
		for key in self.parallel_errors.keys():
			aux += self.parallel_errors[key] * self.gains[key]

		self.output.append(aux)

	def Reset(self):
		""" Reset Errors to Cero"""
		for key in self.parallel_errors.keys():
			self.parallel_errors[key] = 0 

	def Saturate(self):
		self.saturation = True  
		
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