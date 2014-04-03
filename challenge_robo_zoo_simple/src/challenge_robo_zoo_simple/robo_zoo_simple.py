#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')
import rospy

import smach

import robot_smach_states as states
from robot_smach_states.util.startup import startup
import random

from part1 import TurnAround
from look_at_person import LookAtPerson

class RandomOutcome(smach.State):
    """Of the state's registered outcomes, just select a random one"""

    def __init__(self, robot, outcomes):
        smach.State.__init__(self, outcomes=outcomes)

    def execute(self, userdata=None):
        """Randomly selects one of its registered outcomes"""

        return random.choice(self.get_registered_outcomes())


class RoboZooSimple(smach.StateMachine):
    """Calls each part in turn and makes a random switch between parts, i.e smach State(Machine)s.

    ADDING A NEW PART:
    - Create a file with a new thing to do for the challenge, like waving or making a joke.
        This file must contain one main state(machine).
    - Import the file and the stae(machine) from it.
    - Add a new state to the machine below. All of its transitions should make it go back to SELECT_RANDOM
    - Edit the SELECT_RANDOM state to have a new transition to the newly added state.
        The list after 'robot' in its constructor is a list of the possible outcomes and should match with the possible outcomes in its transitions.
        RandomOutcome takes a random item from this list as an outcome."""
    
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        with self:
            smach.StateMachine.add( "SELECT_RANDOM",
                                    RandomOutcome(robot, ["1","2","3"]),
                                    transitions={"1":"SAY_HI",
                                                 "2":"MAKE_JOKES",
                                                 "3":"LOOK_AT_PERSON"})

            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["Howdy", "Hi there"]),
                                    transitions={"spoken":"SELECT_RANDOM"})

            smach.StateMachine.add( "MAKE_JOKES",
                                    states.Say(robot, ["Should I tell a joke?", "Want to hear a robot joke?", "Two robots walk into a bar. Hahahahaha, robots can't walk that well"]),
                                    transitions={"spoken":"SELECT_RANDOM"})

            smach.StateMachine.add( "TURN_AROUND",
                                    TurnAround(robot),
                                    transitions={"Done":"SELECT_RANDOM", "Aborted":"SELECT_RANDOM", "Failed":"SELECT_RANDOM"})

            smach.StateMachine.add( "LOOK_AT_PERSON",
                                    LookAtPerson(robot),
                                    transitions={"Done":"SELECT_RANDOM", "Aborted":"SELECT_RANDOM", "Failed":"SELECT_RANDOM"})

if __name__ == "__main__":
    rospy.init_node("challenge_robo_zoo")

    startup(RoboZooSimple)