#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import os
from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('cocktailparty_rgo2013_tester')

    W = client.SimWorld()

    env = os.environ['ROBOT_ENV']

    if env == "rwc2013":

        person = W.add_object("person-1", "person", 4.1, 0, 0)

        coke = W.add_object("coke-1", "coke", 5.385, -6.455, 0.82)
        milk = W.add_object("milk-1", "milk", 5.385, -7.831, 0.84)     
        apple_juice = W.add_object("apple_juice-1", "apple_juice", 5.385, -7.1, 0.84)  

        W.wait_for_amigo_speech(["What is your name?"])
        W.speak("maxima")
        W.wait_for_amigo_speech(["Is that corect?","Is that OK?", "Is that okay?", "Am I right?", "Is that alright?"])
        W.speak("yes")
        W.wait_for_amigo_speech(["Which drink can I serve you?", "What drink would you like to have?",  "Could you please tell me what drink you want?"])
        W.speak("coke")
        W.wait_for_amigo_speech(["Is that corect?","Is that OK?", "Is that okay?", "Am I right?", "Is that alright?"])
        W.wait_for_amigo_speech([lambda txt: "?" in txt])
        W.speak("yes")
    else:
        print "No test scenario specified for environment {0}.".format(env)
