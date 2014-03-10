#!/usr/bin/env python


import rospy
import rosgraph
import rosgraph.names

ID = '/rosnode'

def get_node_names( namespace = None ):
	""" 
	@param namespace: optional namespace to scope return values by. Namespace must already be resolved.
	@type  namespace: str
	@return: list of node caller IDs
	@rtype: [str]
	@raise ROSNodeIOException: if unable to communicate with master
	"""
	master = rosgraph.Master(ID)
	try:
		state = master.getSystemState()
	except socket.error:
		raise ROSNodeIOException("Unable to communicate with master!")
	nodes = []
	if namespace:
		# canonicalize namespace with leading/trailing slash
		g_ns = rosgraph.names.make_global_ns(namespace)
		for s in state:
			for t, l in s:
				nodes.extend([n for n in l if n.startswith(g_ns) or n == namespace])
	else:
		for s in state:
			for t, l in s:
				nodes.extend(l) 
	return list(set(nodes))

def main():
	nodes = get_node_names( namespace=None )

if __name__ == "__main__": main()
