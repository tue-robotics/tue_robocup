#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cocktail_party')
import rospy

#from tue_execution_pack import states, smach, util, robot_parts
from tue_execution_pack import robot_parts
#from robot_parts.reasoner import *

from psi import *

def do_action(robot, action):
    if action.is_compound():
        if action.get_functor() == 'navigate_to':
            x = float(action[0])
            y = float(action[1])
            phi = float(action[2])
            frame_id = str(action[3])
            
            pos = robot.base.point(x,y)
            orient = robot.base.orient(phi)

            robot.base.send_goal(pos, orient, block=True)
        elif action.get_functor() == 'say':
            robot.speech.speak(str(action[0]))
        elif action.get_functor() == 'listen':
            robot.ears.forget()
            robot.ears.start_listening()
            rospy.sleep(5)
            words = [ str(word) for word in action[0] ]
            print words
            answer = robot.ears.last_heard_words(words, 5)
            robot.ears.stop_listening()
            print answer
            robot.reasoner.assert_fact(Compound("heard_words", str(answer)))
    #print action.functor()
  
if __name__ == '__main__':
    rospy.init_node('executioner')

    robot = robot_parts.amigo.Amigo(wait_services=True)
    
    client = Client("/reasoner/query", "/reasoner/assert")

    client.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
    client.query(Compound("load_database", "challenge_cocktail_party", 'prolog/cocktail_party.pl'))

    client.assert_fact(Compound("challenge", "cocktailparty"))

    finished = False
    while not finished:

        print "* * * * * * * * * * * * * * * * * "
        print ""

        result = client.query(Compound("step", "Actions", "Warnings"))
        if result:
            actions = result[0]["Actions"]
            warnings = result[0]["Warnings"]
            
            if actions.is_sequence():

                for action in actions:
                    print "ACTION: " + str(action)
                    do_action(robot, action)
                print ""
            if warnings.is_sequence():
                for warning in warnings:
                    print "WARNING: " + str(warning)
                print ""

        else:
            print "ERROR: step/2 did not succeed!"
            finished = True

        rospy.sleep(1)
    #print Compound("bla")