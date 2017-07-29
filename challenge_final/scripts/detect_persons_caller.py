#!/usr/bin/env python
import rospy
from image_recognition_msgs.srv import GetPersons
from sensor_msgs.msg import Image


def callback(img):
    try:
        detect_persons(image=img)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
    else:
        rospy.loginfo("Succesfully called detect persons :)")
    rospy.sleep(2.0)


rospy.init_node('detect_persons_caller')
rospy.Subscriber("/amigo/top_kinect/rgb/image", Image, callback, queue_size=1)
print "Waiting for amigo/person_detection/detect_persons"
rospy.wait_for_service('/amigo/person_detection/detect_persons')
detect_persons = rospy.ServiceProxy('/amigo/person_detection/detect_persons', GetPersons)
print "Spinning ..."
rospy.spin()
