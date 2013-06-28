#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

import sys

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_challenge_tester')

    W = client.SimWorld()

    # pantry
    obj1 = W.add_object("drops-1", "drops", 4.529, -4.078, 0.85)

    # couch table
    coke1 = W.add_object("marmalade-2", "marmalade", 8.664, -1.166, 0.5)    

    # large table
    coke2 = W.add_object("tomato-soup-3", "tomato_soup", 5.703, -0.732, 0.8)    
    coke3 = W.add_object("cleaner-4", "cleaner", 5.605, -2.709, 0.85)  

    # bed_cabinet_1
    coke4 = W.add_object("tomato-soup-5", "tomato_soup", 10.229, -5.404, 0.5)    

    # bed_cabinet_2
    coke5 = W.add_object("cleaner-6", "cleaner", 10.219, -7.630, 0.5)    

    # kitchen table
    coke6 = W.add_object("marmalade-7", "marmalade", 3.914, -6.331, 0.8)    
    coke7 = W.add_object("coke-8", "coke", 2.293, -6.324, 0.8)    

    # bedroom cabinet
    coke4 = W.add_object("drops-9", "drops", 6.155, -6.470, 0.5)    
    coke5 = W.add_object("marmalade-10", "marmalade", 6.039, -7.215, 0.5)    

    W.wait_for_amigo_speech(["What can I do for you?"])

    try:
        room = sys.argv[1]
    except IndexError:
        room = "livingroom"

    W.speak("cleanupthe{0}".format(room))
    #W.wait_for_amigo_speech(["Is that corect?","Is that OK?", "Is that okay?", "Am I right?", "Is that alright?"])
    W.wait_for_amigo_speech([lambda txt: "?" in txt])
    rospy.sleep(3.0)
    W.speak("yes")

    W.wait_for_amigo_speech([lambda txt: "done" in txt]) 
    ["I cleaned up everything I could find, so my work here is done. Have a nice day!", "I'm done, everything I could find is cleaned up."]

    rospy.loginfo("Done")
