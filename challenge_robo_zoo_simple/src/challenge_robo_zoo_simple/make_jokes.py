#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')
import rospy 
import smach

jokes = ["Two robots walk into a bar. Hahahahaha, robots can't walk that well",
         "A computer programmer holds up his newly born baby, the mother asks: 'is it a boy or a girl?' He answers: 'yes'."]

class MakeJokes(smach.StateMachine):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot
        
        with self:
            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["Howdy, I got a joke for you", "Hi there, I have a joke for you"]),
                                    transitions={"spoken":"MAKE_JOKES"})

            smach.StateMachine.add( "MAKE_JOKES",
                                    states.Say(robot, jokes),
                                    transitions={"spoken":"Done"})
