#!/usr/bin/env python

import sys, os

import time
import socket

from ed.srv import Query

# XML RPC SERVER
from threading import Thread
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer as Server
import rospy

from std_msgs.msg import String

def error(error):
    print "ERROR: %s"%error
    sys.exit()

class TriggerServer():

    def __init__(self, ip, port):
        self._server = Server((ip, port), allow_none=True)
        self._server.register_function(self.get, 'get')

        self._ros_publisher = rospy.Publisher('/amigo/trigger', String, queue_size=10)

    # RPC METHOD
    def get(self, data):
        self._ros_publisher.publish(String(data=data))

    def serve(self):
        self._server.serve_forever()

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('trigger_server')
        if rospy.has_param('~ip'):
            ip = rospy.get_param('~ip')
            port = rospy.get_param('~port')
            server = TriggerServer(ip, port)
            print "Trigger server active at %s:%d - returns the world model update with use of RPC"%(ip,port)
            server.serve()

        else:
            rospy.logerr("Trigger server : no server ip set; please specify the local 'ip' parameter")
    except rospy.ROSInterruptException:
        pass
