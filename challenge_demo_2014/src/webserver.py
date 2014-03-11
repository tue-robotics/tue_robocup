#!/usr/bin/env python
######
####	Rein Appeldoorn '13
##
#

from SimpleHTTPServer import *
import BaseHTTPServer
import glob
import roslib
import os
import sys

############################
## The simple html server ##
############################

os.chdir(roslib.packages.get_pkg_dir('challenge_demo_2014'))
os.chdir('src/doorbell')

#PORT
sys.argv[1] = '8000'

HandlerClass = SimpleHTTPRequestHandler
ServerClass = BaseHTTPServer.HTTPServer

BaseHTTPServer.test(HandlerClass, ServerClass)
