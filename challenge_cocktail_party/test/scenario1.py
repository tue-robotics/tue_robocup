#!/usr/bin/python
import roslib; roslib.load_manifest('gazebo_life')
import rospy

from gazebo_life import *

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = SimWorld()

    person1 = W.add_object("person-1", "person", 2, 0.2, 1)
    person2 = W.add_object("person-2", "person", 2.081, -0.532, 1)
    person3 = W.add_object("person-3", "person", 2.247, 0.964, 1)
    person4 = W.add_object("person-4", "person", 0.440, 0.193, 1)
    person5 = W.add_object("person-5", "person", 2.179, -1.947, 1)

    raw_input("Press enter to say 'coke'")

    W.speak("coke")

    raw_input("Press enter to say 'yes'")

    W.speak("yes")

    