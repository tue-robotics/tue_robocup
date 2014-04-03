#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')
import rospy

import smach

import robot_smach_states as states
from robot_smach_states.util.startup import startup
import random

from psi import Compound, Conjunction, Sequence
from robot_skills.arms import State as ArmState
from robot_skills.util import msg_constructors as msg
from robot_skills.util import transformations

#from part1 import Part1

class RandomOutcome(smach.State):
    """Of the state's registered outcomes, just select a random one"""

    def __init__(self, robot, outcomes):
        smach.State.__init__(self, outcomes=outcomes)

    def execute(self, userdata=None):
        """Randomly selects one of its registered outcomes"""

        return random.choice(self.get_registered_outcomes())


class RoboZooSimple(smach.StateMachine):
    """Calls each part in turn and makes a random switch between parts, i.e smach State(Machine)s"""
    
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        with self:
            smach.StateMachine.add( "SELECT_RANDOM",
                                    RandomOutcome(robot, ["1","2"]),
                                    transitions={"1":"SAY_HI",
                                                 "2":"MAKE_JOKES"})

            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["Howdy", "Hi there"]),
                                    transitions={"spoken":"SELECT_RANDOM"})

            smach.StateMachine.add( "MAKE_JOKES",
                                    states.Say(robot, ["Should I tell a joke?"]),
                                    transitions={"spoken":"SELECT_RANDOM"})

if __name__ == "__main__":
    rospy.init_node("challenge_robo_zoo")

    startup(RoboZooSimple)