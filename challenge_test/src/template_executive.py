#! /usr/bin/env python

################################################
# Creator: Luis Ferreira (luisfferreira@outlook.com)\
# Maintainers: Loy van Beek (loy.vanbeek@gmail.com) & Luis Ferreira (luisfferreira@outlook.com)
# Date: October 2015
################################################

import rospy
import smach
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.util.startup import startup
import geometry_msgs.msg as gm
import robot_skills.util.msg_constructors as msgs
from robocup_knowledge import load_knowledge

############################## INITIALIZATIONS ##############################

# load knowledge
common_knowledge = load_knowledge("common")
# challenge_knowledge = load_knowledge("challenge_test")

# define print shortcuts from common knowledge
printOk, printError, printWarning = common_knowledge.make_prints("[Challenge Test] ")


############################## MAIN STATE MACHINE ##############################

class ChallengeTemplate(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             TEST CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            testContainer = smach.StateMachine(outcomes = ['container_success', 'container_failed'])
            with testContainer:

                smach.StateMachine.add('TEST_SPEACH',
                                       states.Say(robot,"Testing speach"),
                                       transitions={'spoken':'container_success'})

            # add container to state machine
            smach.StateMachine.add( 'TEST_CONTAINER',
                                    testContainer,
                                    transitions={   'container_success':'END_CHALLENGE',
                                                    'container_failed': 'END_CHALLENGE'})

            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, goodbye!"),
                                   transitions={'spoken':'Done'})


############################## PYTHON ENTRY POINT ##############################

if __name__ == "__main__":
    rospy.init_node('template_executive')


    startup(ChallengeTemplate, challenge_name="template")
