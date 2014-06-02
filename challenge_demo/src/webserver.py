#!/usr/bin/env python

import sys
import os
import signal
import roslib
import rospy

PKG = 'challenge_demo_2014'
roslib.load_manifest(PKG)
os.chdir(roslib.packages.get_pkg_dir(PKG))

default_port = 8000

# frist try to create a twisted server
def createTwistedServer(port):
	from twisted.web import static, server
	from twisted.internet import reactor

	print('using TwistedServer')

	root = static.File('.')
	root.indexNames=['index.html']
	factory = server.Site(root)

	print("Serving HTTP on 0.0.0.0 port %d ...: " % port)
	reactor.listenTCP(port, factory)
	reactor.run()

# if that fails, create a create a SimpleHTTPRequestHandler
def createSimpleHTTPServer(port):
	import SimpleHTTPServer
	import SocketServer

	print('using SimpleHTTPServer')

	Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
	httpd = SocketServer.TCPServer(("", port), Handler)

	print("Serving HTTP on 0.0.0.0 port %d ...: " % port)
	httpd.serve_forever()

if __name__ == '__main__':
	rospy.init_node('webserver', anonymous=True)
	signal.signal(signal.SIGINT, signal.default_int_handler)

	try:
		port = rospy.get_param('~port')
		print('using port %d from parameter server' % port)
	except KeyError:
		print('no parameter server running, getting port from argv')
		if len(sys.argv) > 1 and sys.argv[1].isdigit():
			port = int(sys.argv[1])
		else:
			port = default_port

	try:
		try:
			createTwistedServer(port)
		except ImportError:
			createSimpleHTTPServer(port)
	except KeyboardInterrupt:
		pass
