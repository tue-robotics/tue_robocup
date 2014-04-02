#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('open_challenge_rgo2013_tester')

    W = client.SimWorld()

    table_x = 5.683
    table_y = -2.637

    table = W.add_object("table-1", "table", table_x, table_y, 0)
    W.add_object("seven_up-1",  "seven_up",  table_x - 0.5, table_y + 0.8, 0.88)
    W.add_object("coke-1",  "coke",  table_x - 0.5, table_y + 1.7, 0.88)
    #W.add_object("person-1", "person", 6.309, -1.0, 0)
    W.add_object("person-1", "person", 8.309, -3.0, 0)
#    table = W.add_object("cif-1",  "cif",  table_x + 0.4, table_y + 0.1, 0.92)

#    W.wait_for_amigo_speech("")
    W.wait_for_amigo_speech(["Which drink can I serve you?", "What drink would you like to have?",  "Could you please tell me what drink you want?"])
    W.speak("seven_up")
    W.wait_for_amigo_speech(["Is that corect?","Is that OK?", "Is that okay?", "Am I right?", "Is that alright?"])
    W.wait_for_amigo_speech([lambda txt: "?" in txt])
    rospy.sleep(3.0)
    W.speak("yes")

