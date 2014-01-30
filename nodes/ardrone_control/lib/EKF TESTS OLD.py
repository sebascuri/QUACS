class OdometryGPS(EKF, object):
	def __init__(self, **kwargs):
		self.sensors = [Sensors.GPS()]
		self.processes = [Process.XY_Odometry1( Ts = kwargs.get('Ts', 0) ) , Process.Z_Odometry1( Ts = kwargs.get('Ts', 0) )] 

		super(OdometryGPS, self).__init__(self.sensors, self.processes, **kwargs)

class EKF_3DOF(EKF, object):
	"""docstring for EKF"""
	def __init__(self, ErrorCovariance = np.zeros([3,3]), ProcessCovariance = 0.0003 * np.identity(3), MeasurementCovariance = 0.3  * np.identity(5) , MeasurementJacobian = np.zeros([5,3]), Z = np.array([[0], [0], [0], [0], [0]]), **kwargs):
		super(self).__init__( **kwargs )
		self.ErrorCovariance = ErrorCovariance
		self.ProcessCovariance = ProcessCovariance
		self.MeasurementCovariance = MeasurementCovariance
		self.MeasurementJacobian = MeasurementJacobian
		self.Z = Z

	def update(self):
		super(EKF_3DOF, self).update()
		return np.array([ [self.position.x], [self.position.y], [self.position.yaw]])

	def correct(self):
		super(EKF_3DOF, self).correct()
		self.position.x = self.X[0][0]
		self.position.y = self.X[1][0]
		self.position.yaw = self.X[2][0]

class EKF_6DOF( EKF, object):
	"""docstring for EKF"""
	def __init__(self, ErrorCovariance = np.zeros([6,6]), ProcessCovariance = 0.0003 * np.identity(6), MeasurementCovariance = 0.3  * np.identity(6) , MeasurementJacobian = np.zeros([6,6]), Z = np.array([[0], [0], [0], [0], [0], [0]]), **kwargs):
		super(EKF_6DOF, self).__init__(ErrorCovariance = ErrorCovariance, ProcessCovariance = ProcessCovariance, 
			MeasurementCovariance = MeasurementCovariance, MeasurementJacobian = MeasurementJacobian, Z = Z,  **kwargs )

	def update(self):
		super(EKF_6DOF, self).update()
		self.position.z += self.Ts * self.velocity.z
		 
		aux = np.identity(6)
		aux[0][0:2] = self.Jacobian[0][0:2]
		aux[1][0:2] = self.Jacobian[1][0:2]
		aux[5][5] = self.Jacobian[2][2]
		self.Jacobian = aux
		return np.array([ [self.position.x], [self.position.y], [self.position.z], [self.position.roll], [self.position.pitch], [self.position.yaw]])

	def correct(self):
		super(EKF_6DOF, self).correct()
		self.position.x = self.X[0][0]
		self.position.y = self.X[1][0]
		self.position.z = self.X[2][0]

		self.position.roll = self.X[3][0]
		self.position.pitch = self.X[4][0]
		self.position.yaw = self.X[5][0]