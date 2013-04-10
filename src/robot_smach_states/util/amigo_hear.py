#!/usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
from std_msgs.msg import String
import sys

def talker(sentence = None, keep_looping = False, component = None):
    """ example:
        in src/util
        ./amigo_hear.py "bring me a coke" 0 sentences """
    # add keyword "component" to test "create_speech_files"
    # available components:
    # /speech_appliance/output
    # /speech_bathroomstuff/output
    # /speech_drink/output
    # /speech_food/output
    # /speech_name/output
    # /speech_seat/output
    # /speech_sentences/output
    # /speech_shelf/output
    # /speech_snack/output
    # /speech_table/output
    # /speech_yesno/output

    if component == None:
        pub = rospy.Publisher('/speech/output', String)
    else:
        topic = '/speech_' + component +'/output'
        pub = rospy.Publisher(topic, String)

    rospy.init_node('talker')
    if not sentence:
        sentence = str(raw_input("Your text for AMIGO to hear: "))

    rospy.loginfo(sentence)
    pub.publish(String(sentence))

    if keep_looping:
        rospy.loginfo("Press ctrl-C to stop publishing")
        while not rospy.is_shutdown():
            try:
                if not sentence:
                    loop_sentence = str(raw_input("Your text for AMIGO to hear: "))
                else:
                    loop_sentence = sentence
                rospy.loginfo(loop_sentence)
                pub.publish(String(loop_sentence))
                rospy.sleep(0.5)
            except KeyboardInterrupt: #KeyboardInterrupt
                rospy.loginfo("Quitting now...")
                break

if __name__ == '__main__':
    sentence = None
    if len(sys.argv) >= 2:
        sentence = sys.argv[1]
        if sentence == "":
            sentence = None

    keep_looping = False
    if len(sys.argv) >= 3:
        keep_looping = int(sys.argv[2])

    component = None
    if len(sys.argv) >= 4:
        component = sys.argv[3]

    try:
        talker(sentence, keep_looping, component)
    except rospy.ROSInterruptException: pass
