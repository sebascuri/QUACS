#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy
import tf

class ROS_Object(object):
	"""docstring for ROS"""
	def __init__(self, *args, **kwargs):
		super(ROS_Object, self).__init__()
		if not hasattr(self, 'properties'):
			self.properties = dict()

		self.tfListener = tf.TransformListener()

	@property 
	def timer(self):
		return self.properties.get('timer', None)
	@timer.setter 
	def timer(self, timer):
		self.properties['timer'] = timer
	@timer.deleter
	def timer(self):
		del self.properties['timer']

	@property 
	def subscriber(self):
		return self.properties.get('subscriber', None)
	@subscriber.setter 
	def subscriber(self, subscriber):
		self.properties['subscriber'] = subscriber
	@subscriber.deleter
	def subscriber(self):
		del self.properties['subscriber']

	@property 
	def publisher(self):
		return self.properties.get('publisher', None)
	@publisher.setter 
	def publisher(self, publisher):
		self.properties['publisher'] = publisher
	@publisher.deleter
	def publisher(self):
		del self.properties['publisher']

	@property 
	def services(self):
		return self.properties.get('services', None)
	@services.setter 
	def services(self, services):
		self.properties['services'] = services
	@services.deleter
	def services(self):
		del self.properties['services']

	@property 
	def tf_broadcaster(self):
		return self.properties.get('tf_broadcaster', None)
	@tf_broadcaster.setter 
	def tf_broadcaster(self, tf_broadcaster):
		self.properties['tf_broadcaster'] = tf_broadcaster
	@tf_broadcaster.deleter
	def tf_broadcaster(self):
		del self.properties['tf_broadcaster']

	"""
	@property 
	def tfListener(self):
		return self.properties.get('tfListener', 0)
	@tfListener.setter 
	def tfListener(self, tfListener):
		self.properties['tfListener'] = tfListener
	@tfListener.deleter
	def tfListener(self):
		del self.properties['tf_tfListener']
	"""