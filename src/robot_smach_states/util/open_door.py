#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
from std_msgs.msg import Float32

def opener():
    pub = rospy.Publisher('/middle_point', Float32, queue_size=10)
    
    for distance in [10, 100]:
        rospy.sleep(1.0)
        rospy.loginfo("distance to door: {0}".format(distance))
        pub.publish(Float32(distance))

if __name__ == '__main__':
    rospy.init_node('door_opener')
    try:
        opener()
    except rospy.ROSInterruptException: pass