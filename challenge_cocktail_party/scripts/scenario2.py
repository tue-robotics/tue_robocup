#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_cocktail_party')
import rospy

from fast_simulator.client import *

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = SimWorld()

    raw_input("Press enter to say 'John'")
    W.speak("john")

    raw_input("Press enter to say 'yes'")
    W.speak("yes")

    raw_input("Press enter to say 'coke'")
    W.speak("coke")

    raw_input("Press enter to say 'yes'")
    W.speak("yes")


    raw_input("Press enter to say 'David'")
    W.speak("david")

    raw_input("Press enter to say 'yes'")
    W.speak("yes")

    raw_input("Press enter to say 'fanta'")
    W.speak("fanta")

    raw_input("Press enter to say 'yes'")
    W.speak("yes")

    
