#!/usr/bin/env python

import smach
import rospy
import sys
import smach_ros
import math
from robot_smach_states.util.startup import startup


class LearnOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_follow'])




class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ask_finalize', 'keep_following', 'recover_operator'])
        self.counter = 0

    def execute(self, userdata=None):
        if self.counter < 10:
            if self.counter == 5:
                var = raw_input("Did I lose you? Yes/No: ")
                if var == "No":
                    self.counter += 1
                    return 'keep_following'
                return 'recover_operator'
            self.counter += 1
            return 'keep_following'
        return 'ask_finalize'


class AskFinalize(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['go_follow', 'finalized'])
        self.counter2 = 0

    def execute(self, userdata=None):
        var = raw_input("Am I done following you? Yes/No: ")
        if var == "No":
            return 'go_follow'
        print "Okidoki, we reached the final destination."
        return 'finalized'


class Recovery(smach.State):    
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed', 'go_follow'])

    def execute(self, userdata=None):
        var = raw_input("Did I find you again? Yes/No: ")
        if var == "Yes":
            return 'go_follow'
        print "Oooh noooo, I give up."
        return 'failed'


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['finalized', 'failed'])

    with sm:

        smach.StateMachine.add('FOLLOW', Follow(),
                               transitions={'ask_finalize': 'ASK_FINALIZE',
                                            'keep_following': 'FOLLOW',
                                            'recover_operator': 'RECOVERY'})
        smach.StateMachine.add('ASK_FINALIZE', AskFinalize(),
                               transitions={'go_follow': 'FOLLOW',
                                            'finalized': 'finalized'})
        smach.StateMachine.add('RECOVERY', Recovery(),
                               transitions={'failed': 'failed',
                                            'go_follow': 'FOLLOW'})

        return sm

if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)




