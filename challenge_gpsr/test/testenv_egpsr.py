#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

import sys
import os
import itertools

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_challenge_tester')

    W = client.SimWorld()


    env = os.environ['ROBOT_ENV']

    if env == "robotics_testlabs":

        W.add_object("coke-1","coke", 3.19,4.1,0.85)
        W.add_object("coke-2","coke", 3.14,4.2,0.85)
        W.add_object("coke-3","coke", 3.2,4.3,0.85)
        W.add_object("coke-4","coke", 3.15,4.4,0.85)
        W.add_object("coke-5","coke", 3.17,4.5,0.85)
        W.add_object("coke-6","coke", 3.2,4.6,0.85)


        #person = W.add_object("loy-1", "loy", 0, 4, 0, 0, 0, 0.7)
        W.add_object("sjoerd-2","sjoerd",   2.5, 1.0, 0.000)
        #W.add_object("sjoerd-3","sjoerd",   1, 1, 0.000)
        rospy.sleep(5)

        # person.set_path([   [1.540, -0.021, 0.000],
        #                     [7.031, -0.024, 0.000],
        #                     [7.834, 2.987, 0.000],
        #                     [7.892, 5.444, 0.000],
        #                     [5.526, 7.008, 0.000],
        #                     [4.694, 5.917, 0.000],
        #                     [3.817, 5.120, 0.000],
        #                     [2.995, 4.969, 0.000],
        #                     [2.987, 2.681, 0.000],
        #                     [3.901, 1.687, 0.000],
        #                     [3.229, -0.079, 0.000],
        #                     [-3.084, -0.038, 0.000]], 0.2)  # Last number is path velocity in m/s

    elif env == "rwc2015":

        #W.add_object("coke-1","coke", 3.19,4.1,0.85)
        #W.add_object("coke-2","coke", 3.14,4.2,0.85)
        #W.add_object("coke-3","coke", 3.2,4.3,0.85)
        #W.add_object("coke-4","coke", 3.15,4.4,0.85)
        #W.add_object("coke-5","coke", 3.17,4.5,0.85)
        #W.add_object("coke-6","coke", 3.2,4.6,0.85)


        #person = W.add_object("loy-1", "loy", 0, 4, 0, 0, 0, 0.7)
        W.add_object("sjoerd-1","sjoerd",   3.1, -6.1, 0.000)
        W.add_object("sjoerd-2","sjoerd",   3.2, -6.2, 0.000)
        W.add_object("sjoerd-3","sjoerd",   3.0, -6.2, 0.000)
        W.add_object("sjoerd-4","sjoerd",   3.0, -6.4, 0.000)
        #W.add_object("sjoerd-3","sjoerd",   1, 1, 0.000)
        rospy.sleep(5)




    elif env == "rgo2014":
        coke1 = W.add_object("coke-1", "coke", 4.25, 4.148, 0.86)   

        table_x = 8.028
        table_y = 1.766
        table_vy = 0 #0.03
        table_angle = 0
        table_angle_v = 0#.03   

        # table = W.add_object("dinner_table", "dinner_table", table_x, table_y, 0)        

        # table_x = 5.683
        # table_y = -2.637

        table = W.add_object("table-1", "table", table_x, table_y, 0)


    elif env == "robotics_testlab_B":
        #coke1 = W.add_object("coke-1", "coke", 3.102, 0.0, 0.81)    
        #coke2 = W.add_object("coke-2", "coke", 3.502, -0.5, 0.81) 
        W.add_object("loy-1", "loy", 1.114, 4.233, 0)   

    elif env == "robotics_testlab_A":
        coke2 = W.add_object("coke-2", "coke", 4.8, 2, 0.8)    

        # W.wait_for_amigo_speech(["What can I do for you?"])

        # try:
        #     room = sys.argv[1]
        # except IndexError:
        #     room = "livingroom"

        # W.speak("cleanupthe{0}".format(room))
        # #W.wait_for_amigo_speech(["Is that corect?","Is that OK?", "Is that okay?", "Am I right?", "Is that alright?"])
        # W.wait_for_amigo_speech([lambda txt: "?" in txt])
        # rospy.sleep(3.0)
        # W.speak("yes")

        # W.wait_for_amigo_speech([lambda txt: "done" in txt]) 
        # ["I cleaned up everything I could find, so my work here is done. Have a nice day!", "I'm done, everything I could find is cleaned up."]

        # rospy.loginfo("Done")

    elif env == "rwc2013":

        # pantry
        obj1 = W.add_object("drops-1", "drops", 4.529, -4.078, 0.85)

        # couch table
        coke1 = W.add_object("marmalade-2", "marmalade", 8.664, -1.166, 0.5)    

        # large table
        coke2 = W.add_object("tomato-sauce-3", "tomato_sauce", 5.703, -0.732, 0.8)    
        coke3 = W.add_object("cleaner-4", "cleaner", 5.605, -2.709, 0.85)  

        # bed_cabinet_1
        coke4 = W.add_object("tomato-sauce-5", "tomato_sauce", 10.229, -5.404, 0.5)    

        # bed_cabinet_2
        coke5 = W.add_object("cleaner-6", "cleaner", 10.219, -7.630, 0.5)    

        # kitchen table
        #coke6 = W.add_object("marmalade-7", "marmalade", 3.914, -6.331, 0.8)    
        coke7 = W.add_object("coke-8", "coke", 2.293, -6.324, 0.8)    

        # bedroom cabinet
        coke8 = W.add_object("drops-9", "drops", 6.155, -6.470, 0.5)    
        coke9 = W.add_object("marmalade-10", "marmalade", 6.039, -7.215, 0.5)    

        coke10 = W.add_object("tomato_sauce-11", "tomato_sauce", 9.024, -3.979, 0.5)    
        coke11 = W.add_object("cleaner-12", "cleaner", 8.309, -4.172, 0.5)    



        # W.wait_for_amigo_speech(["What can I do for you?"])

        # try:
        #     room = sys.argv[1]
        # except IndexError:
        #     room = "livingroom"

        # W.speak("cleanupthe{0}".format(room))
        # #W.wait_for_amigo_speech(["Is that corect?","Is that OK?", "Is that okay?", "Am I right?", "Is that alright?"])
        # W.wait_for_amigo_speech([lambda txt: "?" in txt])
        # rospy.sleep(3.0)
        # W.speak("yes")

        # W.wait_for_amigo_speech([lambda txt: "done" in txt]) 
        # ["I cleaned up everything I could find, so my work here is done. Have a nice day!", "I'm done, everything I could find is cleaned up."]

        # rospy.loginfo("Done")




    else:
        print "No test scenario specified for environment {0}.".format(env)


        # pantry
        obj1 = W.add_object("drops-1", "drops", 4.529, -4.078, 0.85)

        # couch table
        coke1 = W.add_object("marmalade-2", "marmalade", 8.664, -1.166, 0.5)    

        # large table
        coke2 = W.add_object("tomato-sauce-3", "tomato_sauce", 5.703, -0.732, 0.8)    
        coke3 = W.add_object("cleaner-4", "cleaner", 5.605, -2.709, 0.85)  

        # bed_cabinet_1
        coke4 = W.add_object("tomato-sauce-5", "tomato_sauce", 10.229, -5.404, 0.5)    

        # bed_cabinet_2
        coke5 = W.add_object("cleaner-6", "cleaner", 10.219, -7.630, 0.5)    

        # kitchen table
        #coke6 = W.add_object("marmalade-7", "marmalade", 3.914, -6.331, 0.8)    
        coke7 = W.add_object("coke-8", "coke", 2.293, -6.324, 0.8)    

        # bedroom cabinet
        coke8 = W.add_object("drops-9", "drops", 6.155, -6.470, 0.5)    
        coke9 = W.add_object("marmalade-10", "marmalade", 6.039, -7.215, 0.5)    

        coke10 = W.add_object("tomato_sauce-11", "tomato_sauce", 9.024, -3.979, 0.5)    
        coke11 = W.add_object("cleaner-12", "cleaner", 8.309, -4.172, 0.5)    



    # W.wait_for_amigo_speech(["What can I do for you?"])

    # try:
    #     room = sys.argv[1]
    # except IndexError:
    #     room = "livingroom"

    # W.speak("cleanupthe{0}".format(room))
    # #W.wait_for_amigo_speech(["Is that corect?","Is that OK?", "Is that okay?", "Am I right?", "Is that alright?"])
    # W.wait_for_amigo_speech([lambda txt: "?" in txt])
    # rospy.sleep(3.0)
    # W.speak("yes")

    # W.wait_for_amigo_speech([lambda txt: "done" in txt]) 
    # ["I cleaned up everything I could find, so my work here is done. Have a nice day!", "I'm done, everything I could find is cleaned up."]

    # rospy.loginfo("Done")
