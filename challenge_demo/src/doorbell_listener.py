#!/usr/bin/env python
import os
import subprocess as sub

import roslib
import rospy
import rosnode
from std_msgs.msg import String
import diagnostic_msgs.msg

PKG = 'challenge_demo'
roslib.load_manifest(PKG)
os.chdir(roslib.packages.get_pkg_dir(PKG))

def callback(data): 
    if(data.data == "doorbell"):
        response = sub.Popen(['aplay','src/doorbell.wav'])

# Main function.
if __name__ == '__main__':
    rospy.init_node('doorbell_listener')
    subscriber = rospy.Subscriber("/trigger", String, callback)
    while not rospy.is_shutdown():
        rospy.sleep(.5)	
