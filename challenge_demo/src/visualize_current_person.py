#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_demo')
import rospy;

import visualization_msgs.msg

from psi import *

if __name__ == '__main__':

    rospy.init_node('visualize_current_person')

    ''' Reasoner '''
    client = Client("/reasoner")
    term   = Conjunction(Compound("current_person", "ObjectID"),
                         Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

    ''' Marker publisher '''
    marker_pub = rospy.Publisher("/visualization_marker", visualization_msgs.msg.Marker)

    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = "/map"
    marker.type = 3
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.lifetime = rospy.Duration(1.0)
    
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 1.8
    
    marker.pose.position.z = marker.scale.z/2

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.25;

    while not rospy.is_shutdown():

        ''' Query reasoner '''
        answers = client.query(term)
        if answers:
            answer = answers[0]
            marker.pose.position.x = answer["X"]
            marker.pose.position.y = answer["Y"]
            #rospy.loginfo("ID = {0}".format(answer["ObjectID"]))

            ''' Publish marker '''
            marker.header.stamp = rospy.Time.now()
            marker_pub.publish(marker)
            #rospy.loginfo("Person found, publishing: {0}".format(marker))
        #else:
            #rospy.loginfo("No person found yet")
            
        rospy.sleep(rospy.Duration(0.1))
