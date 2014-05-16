#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')
import rospy 
import smach

import robot_smach_states as states
from look_at_person import LookAtPerson

jokes = ["Two robots walk into a bar. Hahahahaha, robots can't walk that well",
         "A computer programmer holds up his newly born baby, the mother asks: 'is it a boy or a girl?' He answers: 'yes'.",
         "I once dated a MacBook. It didn't work, because she was all 'I' this and 'I' that."]

class MakeJokes(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done"])
        self.robot = robot
        
        with self:
            smach.StateMachine.add("LOOK_AT_PERSON",
                    LookAtPerson(robot),
                    transitions={   'Done'      :'SAY_HI',
                                    'Aborted'   :'SAY_HI',
                                    'Failed'    :'SAY_HI'})

            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["Howdy, I got a joke for you", "Hi there, I have a joke for you"]),
                                    transitions={"spoken":"MAKE_JOKES"})

            smach.StateMachine.add( "MAKE_JOKES",
                                    states.Say(robot, jokes),
                                    transitions={"spoken":"Done"})
