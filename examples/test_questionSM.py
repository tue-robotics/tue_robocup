#! /usr/bin/env python

import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach

import robot_smach_states as states
import robot_smach_states.util as util
from psi  import Compound

class Test_QuestionMachine(smach.StateMachine):
    def __init__(self, robot):
        r = robot.reasoner
        r.reset()
        
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])
        
        with self:
            smach.StateMachine.add( "ASK_QUESTION", 
                                    states.Timedout_QuestionMachine(robot=robot,
                                                     sentence="What do you want to clean up?",
                                                     options={  'cleanupthelivingroom':Compound("cleaning", "livingroom"), 
                                                                'cleanupthebedroom':Compound("cleaning", "bedroom"), 
                                                                'cleanupthelobby':Compound("cleaning", "lobby"),
                                                                'cleanupthekitchen':Compound("cleaning", "kitchen")},
                                                     default_option='coke'),
                                    transitions={'answered':'RECITE_ORDER',
                                                 'not_answered':'RECITE_ORDER'},
                                    remapping={'answer':'sentence'})
    #          
            def query_desired_drink(userdata):
                answers = r.query(r.cleaning("Room"))
                if answers:
                    return "I will clean the {0}".format(answers[0]["Room"])
                else:
                    return "I did not remember what to clean"

            smach.StateMachine.add( "RECITE_ORDER",
                                    states.Say_generated(robot, sentence_creator=query_desired_drink),
                                    transitions={'spoken':'Done'})

############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('executioner')
    
    util.startup(Test_QuestionMachine)
