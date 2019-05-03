#!/usr/bin/python

"""
Clean UP [Housekeeper] challenge
This challenge is described in the 2019 RoboCup@Home Rulebook / Draft version

Main goal
Upon entrance, the robot requests the operator which room shall be cleaned. All misplaced known objects
found in this room must be taken to their predefined locations and unknown objects thrown in the trash bin.

Timelimit: 5 minutes

Number of objects: 5 to 10
Objects can be anywhere, including the floor, seats, and on furniture. All objects are visible from
at least 1.0 m distance (no occlusions) and have the following distributions:
    Known objects: Any two regular and two alike objects
    Unknown objects: One unknown object at grasping distance (i.e. no decorations)

Reward: 1000 pts (100 pts per object)
Bonus: max 500 pts

Adapted from the r5cop_demo challenge (see the repo)

Difference from goal:
- Robot does not inspect the floor, Can we dynamically inspect the floor, or must we stop to inspect?
- Limited inspection of cabinets
- trash_bin and trash_can not handled correctly as disposers. Only one of the two.


"""

import rospy
import smach
import random

import sys

import hmi

import robot_smach_states
from robot_smach_states.util.designators import VariableDesignator, VariableWriter, EntityByIdDesignator
from clean_inspect import CleanInspect
from robot_smach_states.utility import SetInitialPose

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_cleanup')

from robot_skills import robot

class VerifyWorldModelInfo(smach.State):
    """
    Check consistency between world model and local knowledge
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["failed", "done"])
        self._robot = robot

    def execute(self, userdata):
    # Look for trash units
        ids = [e.id for e in self._robot.ed.get_entities()]
        for loc in challenge_knowledge.cleaning_locations:
            if loc["room"] == "living_room":
                if "trash_bin" not in ids:
                    return "failed"
            if loc["room"] == "kitchen":
                if "trash_can" not in ids:
                    return "failed"

    # Make sure the world model and local knowledge match
        for place in challenge_knowledge.cleaning_locations:
            if place["name"] not in ids:
                return "failed"
            if place["room"] not in ids:
                return "failed"

        return "done"

def collect_cleanup_entities(room):
    """
    Create list of points to visit in the selected room
    :param room:
    :return:
    """
    cleaning_locations = []
    for loc in challenge_knowledge.cleaning_locations:
        if loc["room"] == room:
            cleaning_locations.append(loc)
    return cleaning_locations



def setup_statemachine(robot, room):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    robot.ed.reset()

    # Show object locations in designated room
    cleaning_locations = collect_cleanup_entities(room)
    rospy.loginfo("Cleaning locations: {}".format(cleaning_locations))

    with sm:

        # smach.StateMachine.add( "INITIALIZE",
        #                         robot_smach_states.Initialize(robot),
        #                         transitions={ "initialized"   :"SET_INIT", "abort"         :"Aborted"})

        smach.StateMachine.add("SET_INIT",
                               SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'VERIFY',
                                                'preempted': 'Aborted',
                                                'error': 'VERIFY'})

        smach.StateMachine.add('VERIFY',
                                VerifyWorldModelInfo(robot),
                                transitions={"done": "SAY_START_CHALLENGE", "failed" : "SAY_KNOWLEDGE_NOT_COMPLETE"})

        smach.StateMachine.add('SAY_KNOWLEDGE_NOT_COMPLETE',
                               robot_smach_states.Say(robot, ["My knowledge of the world is not complete!",
                                                              "Please give me some more information!"], block=False),
                               transitions={"spoken": "Aborted"})

        smach.StateMachine.add('SAY_START_CHALLENGE',
                               robot_smach_states.Say(robot, ["Starting the cleanup challenge",
                                                              "What a mess here, let's clean this room!",
                                                              "Let's see if I can find some garbage here",
                                                              "All I want to do is clean this mess up!"], block=False),
                               transitions={"spoken": "INSPECT_0"})

        for i, place in enumerate(cleaning_locations):
            next_state = "INSPECT_%d" % (i + 1) if i + 1 < len(cleaning_locations) else "RETURN_TO_OPERATOR"

            smach.StateMachine.add("INSPECT_%d" % i,
                                   CleanInspect(robot, place["name"], place["room"], place["navigate_area"],
                                                place["segment_areas"]),
                                   transitions={"done": next_state})

        smach.StateMachine.add("RETURN_TO_OPERATOR",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                                 id=challenge_knowledge.starting_point),
                                                                     radius=0.3),
                               transitions={"arrived": "SAY_CLEANED_ROOM",
                                            "unreachable": "SAY_CLEANED_ROOM",
                                            "goal_not_defined": "SAY_CLEANED_ROOM"})

        smach.StateMachine.add('SAY_CLEANED_ROOM',
                               robot_smach_states.Say(robot, ["I successfully cleaned the {}!".format(room),
                                                              "Am I a good robot now?",
                                                              "There, I cleaned up your mess, are you happy now!"], block=False),
                               transitions={"spoken": "Done"})

    return sm

def confirm(robot):
    cgrammar = """
    C[P] -> A[P]
    A['yes'] -> yes
    A['no'] -> no
    """
    try:
        speech_result = robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;"
                                                                                        "T[False] -> no", target="T")
    except TimeoutException:
        return False

    return speech_result.semantics

def ask_which_room_to_clean(robot):
    max_tries = 5
    nr_of_tries = 0
    count = 0

    robot.head.look_at_standing_person(3)

    while nr_of_tries < max_tries and not rospy.is_shutdown():
        while not rospy.is_shutdown():
            count += 1
            robot.speech.speak("Which room should I clean for you?", block=True)
            try:
                speech_result = robot.hmi.query(description="",
                                                           grammar=challenge_knowledge.grammar,
                                                           target="T")
                rospy.loginfo("sentence: {}".format(speech_result.sentence))
                rospy.loginfo("semantics: {}".format(speech_result.semantics))
                break
            except (hmi.TimeoutException, hmi.GoalNotSucceededException) as e:
                if count < 5:
                    robot.speech.speak(random.choice(["I'm sorry, can you repeat",
                                                     "Please repeat, I didn't hear you",
                                                     "I didn't get that can you repeat it",
                                                     "Please speak up, as I didn't hear you"]))
                else:
                    robot.speech.speak("I am sorry but I cannot understand you. I will quit now", block=False)
                    robot.head.cancel_goal()
                    return "failed"

        try:
            # Now: confirm
            robot.speech.speak("I understood that the {} should be cleaned "
                                         "is this correct?".format(speech_result.sentence))
        except:
            continue

        if confirm(robot):
            robot.head.cancel_goal()
            robot.speech.speak("Ok, I will clean the {}".format(speech_result.sentence), block=False)
            return speech_result.sentence

        nr_of_tries += 1



def main():
    rospy.init_node('cleanup_challenge')

    skip = rospy.get_param('~skip', False)
    robot_name = rospy.get_param('~robot_name')

    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        raise ValueError('unknown robot')

    robot = Robot()

    # Wait for door, enter arena
    #  skip is defined and set in the launchfile. (TRUE)
    if not skip:
        #s = StartChallengeRobust(robot, challenge_knowledge.initial_pose)
        #s.execute()

        robot.speech.speak("Moving to the meeting point.", block=False)
        nwc = robot_smach_states.NavigateToWaypoint(robot=robot,
                                 waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                          id=challenge_knowledge.starting_point),
                                 radius=0.3)
        nwc.execute()

    room = ask_which_room_to_clean(robot)

    # start execution of challenge
    sm_args = [room]
    robot_smach_states.util.startup(setup_statemachine, challenge_name="cleanup", statemachine_args=sm_args)


if __name__ == '__main__':
    main()
