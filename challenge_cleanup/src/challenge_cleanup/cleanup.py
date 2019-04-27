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

import robot_smach_states
from robot_smach_states.util.designators import VariableDesignator, VariableWriter
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

        for place in challenge_knowledge.inspection_places:
            if place["entity_id"] not in ids:
                return "failed"
            if place["room_id"] not in ids:
                return "failed"

        return "done"

def collect_cleanup_entities(room_des):
    room = room_des.resolve()
    cleaning_locations = []
    for loc in challenge_knowledge.cleaning_locations:
        if loc["room"] == room:
            cleaning_locations.append(loc)
    return cleaning_locations



def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    robot.ed.reset()

    # temporary room hardcoded: replace with speech input
    room = VariableDesignator("livingroom")
    room_writer = VariableWriter(room)
    room_writer.write("kitchen")
    rospy.loginfo("cleaning room %s" %room.resolve())

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

if __name__ == '__main__':
    rospy.init_node('cleanup_challenge')

    # Check if we have something specified to inspect
    if len(challenge_knowledge.inspection_places) < 1:
        rospy.logerr("The challenge knowledge inspection_places list should contain at least one entry!")
        sys.exit(1)

    robot_smach_states.util.startup(setup_statemachine, challenge_name="cleanup")
