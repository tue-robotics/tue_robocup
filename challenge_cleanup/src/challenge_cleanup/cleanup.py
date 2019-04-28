#!/usr/bin/python

"""
Clean UP [Housekeeper] challenge
This challenge is described in the 2019 RoboCup@Home Rulebook / Draft version

Main goal
Upon entrance, the robot requests the operator which room shall be cleaned. All misplaced known objects
found in this room must be taken to their predefined locations and unknown objects thrown in the trash bin.

Number of objects: 5 to 10
Objects can be anywhere, including the floor, seats, and on furniture. All objects are visible from
at least 1.0 m distance (no occlusions) and have the following distributions:
    Known objects: Any two regular and two alike objects
    Unknown objects: One unknown object at grasping distance (i.e. no decorations)

Reward: 1000 pts (100 pts per object)
Bonus: max 500 pts

Adapted from the r5cop_demo challenge (see the repo)
"""

import rospy
import smach

import sys

import hmi

import robot_smach_states
from robot_smach_states.util.designators import VariableDesignator, VariableWriter, EntityByIdDesignator
from clean_inspect import CleanInspect

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_cleanup')

class VerifyWorldModelInfo(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["failed", "done"])
        self._robot = robot

    def execute(self, userdata):

        ids = [e.id for e in self._robot.ed.get_entities()]
        if "trashbin" not in ids:
            return "failed"

        for place in challenge_knowledge.cleaning_locations:
            if place["name"] not in ids:
                return "failed"
            if place["room"] not in ids:
                return "failed"

        return "done"

def collect_cleanup_entities(room):
    cleaning_locations = []
    for loc in challenge_knowledge.cleaning_locations:
        if loc["room"] == room:
            cleaning_locations.append(loc)
    return cleaning_locations



def setup_statemachine(robot, room):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    robot.ed.reset()

    cleaning_locations = collect_cleanup_entities(room)
    rospy.loginfo("Cleaning locations: {}".format(cleaning_locations))

    with sm:

        smach.StateMachine.add( "INITIALIZE",
                                robot_smach_states.Initialize(robot),
                                transitions={ "initialized"   :"VERIFY", "abort"         :"Aborted"})

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
            next_state = "INSPECT_%d" % (i + 1) if i + 1 < len(cleaning_locations) else "Done"

            smach.StateMachine.add("INSPECT_%d" % i,
                                   CleanInspect(robot, place["name"], place["room"], place["navigate_area"],
                                                place["segment_areas"]),
                                   transitions={"done": next_state})
    return sm

def ask_which_room_to_clean(robot):

    robot.speech.speak("Which room should I clean for you?", block=True)
    try:
        sentence, semantics = robot.hmi.query(description="",
                                                   grammar=challenge_knowledge.grammar,
                                                   target="T")
        rospy.loginfo("sentence: {}".format(sentence))
        rospy.loginfo("semantics: {}".format(semantics))
        return sentence
    except (hmi.TimeoutException, hmi.GoalNotSucceededException) as e:
        rospy.logwarn("HMI failed when asking for room: {}".format(e))



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
    if not skip:
        #s = StartChallengeRobust(robot, challenge_knowledge.initial_pose)
        #s.execute()

        robot.speech.speak("Moving to the meeting point.", block=False)
        nwc = robot_smach_states.NavigateToWaypoint(robot=robot,
                                 waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                          id=challenge_knowledge.starting_pose),
                                 radius=0.3)
        nwc.execute()

    room = ask_which_room_to_clean(robot)

    # start execution of challenge
    sm_args = [room]
    robot_smach_states.util.startup(setup_statemachine, challenge_name="cleanup", statemachine_args=sm_args)


if __name__ == '__main__':
    main()
