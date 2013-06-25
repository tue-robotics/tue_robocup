#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_challenge_tester')

    W = client.SimWorld()

    # couch table
    coke1 = W.add_object("coke-1", "coke", 8.664, -1.166, 0.5)    

    # large table
    coke2 = W.add_object("cif-2", "cif", 5.703, -0.732, 0.8)    
    coke3 = W.add_object("tea_pack-3", "tea_pack", 5.605, -2.709, 0.8)  

    # bed_cabinet_1
    coke4 = W.add_object("coke-4", "coke", 10.229, -5.404, 0.5)    

    # bed_cabinet_2
    coke5 = W.add_object("rubiks-5", "rubiks", 10.219, -7.630, 0.5)    

    # kitchen table
    coke6 = W.add_object("coke-6", "coke", 3.914, -6.331, 0.8)    
    coke7 = W.add_object("coke-7", "coke", 2.293, -6.324, 0.8)    

    # bedroom cabinet
    coke4 = W.add_object("coke-8", "coke", 6.155, -6.470, 0.5)    
    coke5 = W.add_object("coke-9", "coke", 6.039, -7.215, 0.5)    

    W.wait_for_amigo_speech(["What can I do for you?"])
    W.speak("cleanupthebedroom")
    W.wait_for_amigo_speech(["Is that corect?","Is that OK?"])
    W.speak("yes")

    W.wait_for_amigo_speech(["I cleaned up everything I could find, so my work here is done. Have a nice day!", "I'm done, everything I could find is cleaned up."])

    rospy.loginfo("Done")