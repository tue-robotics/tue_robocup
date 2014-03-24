#!/usr/bin/env python

import roslib; roslib.load_manifest('challenge_demo_2014')
import rospy
import rosnode
from std_msgs.msg import String
import diagnostic_msgs.msg
import subprocess as sub

def callback(data): 
    if(data.data == "doorbell"):
        response = sub.Popen(['aplay','doorbell.wav'])

# Main function.
if __name__ == '__main__':
    rospy.init_node('node_alive_server')
    subscriber = rospy.Subscriber("/trigger", String, callback)
    while not rospy.is_shutdown():
        rospy.sleep(.5)	
