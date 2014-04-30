#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')
import rospy

import smach

import robot_smach_states as states
from robot_smach_states.util.startup import startup
import random

from part1 import TurnAround
from look_at_person import LookAtPerson
from flash_lights import FlashLights
from walk_like_an_egyptian import WalkLikeAnEgyptian
from boo import Boo

from demo_executioner import wave_lights #amigo_demo package is not using the recommended package layout with amigo_demo/src/amigo-demo


class RandomOutcome(smach.State):
    """Of the state's registered outcomes, just select a random one"""

    def __init__(self, robot, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        self.previous_outcome = None

    def execute(self, userdata=None):
        """Randomly selects one of its registered outcomes. It excludes the previous outcome so it doesn't do the same thing twice"""
        possible_outcomes = list(self.get_registered_outcomes())
        if self.previous_outcome in possible_outcomes:
            possible_outcomes.remove(self.previous_outcome)
        current_outcome = random.choice(possible_outcomes)
        self.previous_outcome = current_outcome
        return current_outcome


class RoboZooSimple(smach.StateMachine):
    """Calls each part in turn and makes a random switch between parts, i.e smach State(Machine)s.

    ADDING A NEW PART:
    - Create a file with a new thing to do for the challenge, like waving or making a joke.
        This file must contain one main state(machine).
    - Import the file and the stae(machine) from it.
    - Add a new state to the machine below. All of its transitions should make it go back to RESET_ALL
    - Edit the SELECT_RANDOM state to have a new transition to the newly added state.
        The list after 'robot' in its constructor is a list of the possible outcomes and should match with the possible outcomes in its transitions.
        RandomOutcome takes a random item from this list as an outcome.

    It could be handy 

    IDEAS:
        - DONE: Reset everything before SELECT_RANDOM
        - DONE: Walk like an egyptian (with arms and music)
        - DONE: Flash lights
        - DONE: wave (smile and wave boys)
        - TEST: Look at person and say something funny
        - Act like a monkey (with arms and sound)
        - Use AR markers to select actions
        - Photo pose with speech
        - DONE: Stare into the distance and wait for a face to appear. Then say "boo!" :-)
        - Play the tune from jaws when you are not looking and stop when you are looking.
        - Macarena move
        - Do movie-quotes and sounds:
            - "I'll be back" from Terminator
            - Roger Roger from the star wars battle droids
            - Play the Mos Eisly pub music
        - When Amigo recognizes a face, say 'These aren't the human I am looking for' and do the jedi arm wiggle
        - Wait for 'stop' to be said and say "Hammertime"

    """
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
        with self:
            
            smach.StateMachine.add( "RESET_ALL",
                                    states.ResetArmsSpindleHead(robot),
                                    transitions={"done":"SELECT_RANDOM"})

            smach.StateMachine.add( "SELECT_RANDOM",
                                    RandomOutcome(robot, ["1","2","3","4","5","6", "7"]),
                                    transitions={"1":"SAY_HI",
                                                 "2":"MAKE_JOKES",
                                                 "3":"LOOK_AT_PERSON",
                                                 "4":"FLASH_LIGHTS",
                                                 "5":"WAVE_LIGHTS",
                                                 "6":"WALK_EGYPTIAN",
                                                 "7":"BOO"})

            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["Howdy", "Hi there"]),
                                    transitions={"spoken":"RESET_ALL"})

            smach.StateMachine.add( "MAKE_JOKES",
                                    states.Say(robot, ["Two robots walk into a bar. Hahahahaha, robots can't walk that well",
                                                       "A computer programmer holds up his newly born baby, the mother asks: 'is it a boy or a girl?' He answers 'yes'."]),
                                    transitions={"spoken":"RESET_ALL"})

            smach.StateMachine.add( "TURN_AROUND",
                                    TurnAround(robot),
                                    transitions={"Done":"RESET_ALL", "Aborted":"RESET_ALL", "Failed":"RESET_ALL"})

            smach.StateMachine.add( "LOOK_AT_PERSON",
                                    LookAtPerson(robot),
                                    transitions={"Done":"RESET_ALL", "Aborted":"RESET_ALL", "Failed":"RESET_ALL"})

            smach.StateMachine.add( "FLASH_LIGHTS",
                                    FlashLights(robot),
                                    transitions={"Done":"RESET_ALL"})
            
            smach.StateMachine.add( "WALK_EGYPTIAN",
                                    WalkLikeAnEgyptian(robot),
                                    transitions={"Done":"RESET_ALL"})
            
            smach.StateMachine.add( "BOO",
                                    Boo(robot),
                                    transitions={"Done":"RESET_ALL", "Aborted":"RESET_ALL", "Failed":"RESET_ALL"})
            
            @smach.cb_interface(outcomes=['done'])
            def wave_lights_wrapped(*args, **kwargs):
                wave_lights(robot)
                return 'done'
            smach.StateMachine.add( "WAVE_LIGHTS",
                                    smach.CBState(wave_lights_wrapped),
                                    transitions={"done":"RESET_ALL"})

if __name__ == "__main__":
    rospy.init_node("challenge_robo_zoo")

    startup(RoboZooSimple)