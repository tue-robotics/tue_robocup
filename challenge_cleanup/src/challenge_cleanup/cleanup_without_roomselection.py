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
#ToDo: iterating over designators, such as a dictionary, is NOT possible. This means that choosing the
#ToDo: room to cleanup cannot be done from within the state machine. Major Bummer!!!
#Choosing before entering the state machine seems the easiest option to implement, but is not
#really satisfactory. (see previous version of this module)

import rospy
import smach
import random

import sys

import hmi

import robot_smach_states
from robot_smach_states.util.designators import VariableDesignator, VariableWriter, EntityByIdDesignator
from clean_inspect import CleanInspect
from robot_smach_states.utility import SetInitialPose

# Try the new designator iterator (2019-05-14)
from robot_smach_states import designator_iterator

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

class AskWhichRoomToClean(smach.State):

    def __init__(self, robot, roomw, answerw, cleanup_locationsw):
        smach.State.__init__(self, outcomes=["failed", "done"])
        self.robot = robot
        self.roomw = roomw
        self.answerw = answerw
        self.cleanup_locationsw = cleanup_locationsw

    def collect_cleanup_locations(self):
        cleaning_locations = []
        for loc in challenge_knowledge.cleaning_locations:
            if loc["room"] == self.roomw.resolve():
                cleaning_locations.append(loc)
        self.cleanup_locationsw.write(cleaning_locations)
        rospy.loginfo("Cleaning locations: {}".format(self.cleanup_locationsw.resolve()))
        return

    def execute(self, userdata):
        max_tries = 5
        nr_of_tries = 0
        count = 0

        self.robot.head.look_at_standing_person(3)

        while nr_of_tries < max_tries and not rospy.is_shutdown():
            while not rospy.is_shutdown():
                count += 1
                self.robot.speech.speak("Which room should I clean for you?", block=True)
                try:
                    speech_result = self.robot.hmi.query(description="",
                                                               grammar=challenge_knowledge.grammar,
                                                               target="T")
                    rospy.loginfo("sentence: {}".format(speech_result.sentence))
                    rospy.loginfo("semantics: {}".format(speech_result.semantics))
                    self.roomw.write(speech_result.sentence)
                    rospy.loginfo("roomw: {}".format(self.roomw.resolve()))
                    break
                except (hmi.TimeoutException, hmi.GoalNotSucceededException) as e:
                    if count < 5:
                        self.robot.speech.speak(random.choice(["I'm sorry, can you repeat",
                                                         "Please repeat, I didn't hear you",
                                                         "I didn't get that can you repeat it",
                                                         "Please speak up, as I didn't hear you"]))
                    else:
                        self.robot.speech.speak("I am sorry but I cannot understand you. I will quit now", block=False)
                        self.robot.head.cancel_goal()
                        return "failed"

            try:
                # Now: confirm
#                self.roomw.write(speech_result.sentence)
                self.robot.speech.speak("I understood that the {} should be cleaned "
                                             "is this correct?".format(speech_result.sentence))
            except:
                continue

            try:
                speech_result = self.robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;"
                                                                                            "T[False] -> no",
                                                                                     target="T")
            except TimeoutException:
                return "failed"

            self.robot.head.cancel_goal()
#            self.robot.speech.speak("Ok, I will clean the {}".format(speech_result.sentence), block=False)
            self.robot.speech.speak("Ok, I will clean the {}".format(self.roomw.resolve()), block=False)
            self.collect_cleanup_locations()


            return "done"

        nr_of_tries += 1

# def collect_cleanup_entities(room):
#     """
#     Create list of points to visit in the selected room
#     :param room:
#     :return:
#     """
#     cleaning_locations = []
#     for loc in challenge_knowledge.cleaning_locations:
#         if loc["room"] == room:
#             cleaning_locations.append(loc)
#     return cleaning_locations


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    robot.ed.reset()
    # Designators
    roomr = VariableDesignator('kitchen', resolve_type=str)
    roomw =roomr.writeable
    answerr = VariableDesignator('yes', resolve_type=str)
    answerw = VariableWriter(answerr)
    cleanup_locationsr = VariableDesignator([{'1':'2','3':'4'}])
    cleanup_locationsw = cleanup_locationsr.writeable

    # Show object locations in designated room
    # cleaning_locations = collect_cleanup_entities(room)
#    cleaning_locations = collect_cleanup_entities(roomr.resolve())
#    rospy.loginfo("Cleaning locations: {}".format(cleanup_locationsr))

    with sm:

        # Start challenge via StartChallengeRobust
        # smach.StateMachine.add("START_CHALLENGE_ROBUST",
        #                        robot_smach_states.StartChallengeRobust(robot,challenge_knowledge.initial_pose),
        #                        transitions={"Done":    "GO_TO_START",
        #                                     "Aborted": "GO_TO_START",
        #                                     "Failed":  "GO_TO_START"})
        # smach.StateMachine.add("GO_TO_START",
        #                        robot_smach_states.NavigateToWaypoint(robot=robot,
        #                                                              waypoint_designator=EntityByIdDesignator(robot=robot,
        #                                                                          id=challenge_knowledge.starting_point),
        #                                                              radius=0.3),
        #                        transitions={"arrived": "INQUIRE_ROOM",
        #                                     "unreachable": "INQUIRE_ROOM",
        #                                     "goal_not_defined": "INQUIRE_ROOM"})


        # The next two states 'teleport' the robot from the initial_pose to the challenge starting point
        # in the arena. These states replace the two states above, which do not work correctly.

        smach.StateMachine.add("INITIALIZE",
                               robot_smach_states.Initialize(robot),
                               transitions={"initialized": "SET_INITIAL_POSE",
                                            "abort": "Aborted"})
        smach.StateMachine.add("SET_INITIAL_POSE",
                               robot_smach_states.SetInitialPose(robot,challenge_knowledge.starting_point),
                               transitions={"done":    "INQUIRE_ROOM",
                                            "preempted": "Aborted",
                                            "error":  "INQUIRE_ROOM"})

        smach.StateMachine.add("INQUIRE_ROOM",
                                AskWhichRoomToClean(robot, roomw, answerw, cleanup_locationsw),
                                transitions={"done":    "VERIFY",
                                             "failed":  "INQUIRE_ROOM"})

        smach.StateMachine.add('VERIFY',
                           VerifyWorldModelInfo(robot),
                           transitions={"done": "Done", "failed": "SAY_KNOWLEDGE_NOT_COMPLETE"})

        smach.StateMachine.add('SAY_KNOWLEDGE_NOT_COMPLETE',
                           robot_smach_states.Say(robot, ["My knowledge of the world is not complete!",
                                                          "Please give me some more information!"], block=False),
                           transitions={"spoken": "Aborted"})

        smach.StateMachine.add('SAY_START_CHALLENGE',
                               robot_smach_states.Say(robot, ["Starting the cleanup challenge",
                                                              "What a mess here, let's clean this room!",
                                                              "Let's see if I can find some garbage here",
                                                              "All I want to do is clean this mess up!"], block=False),
#                               transitions={"spoken": "INSPECT_0"})
                               transitions={"spoken": "RETURN_TO_OPERATOR"})

        # for i, place in enumerate(cleanup_locationsr):
        #     next_state = "INSPECT_%d" % (i + 1) if i + 1 < len(cleanup_locationsr) else "RETURN_TO_OPERATOR"
        #
        #     smach.StateMachine.add("INSPECT_%d" % i,
        #                            CleanInspect(robot, place["name"], place["room"], place["navigate_area"],
        #                                              place["segment_areas"]),
        #                            transitions={"done": next_state})
        #
        smach.StateMachine.add("RETURN_TO_OPERATOR",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                                 id=challenge_knowledge.starting_point),
                                                                     radius=0.3),
                               transitions={"arrived": "SAY_CLEANED_ROOM",
                                            "unreachable": "SAY_CLEANED_ROOM",
                                            "goal_not_defined": "SAY_CLEANED_ROOM"})

        smach.StateMachine.add('SAY_CLEANED_ROOM',
                               robot_smach_states.Say(robot, ["I successfully cleaned the {}!".format(roomr),
                                                              "All done. Am I a good robot now?",
                                                              "There, I cleaned up your mess, are you happy now!"], block=False),
                               transitions={"spoken": "Done"})

    return sm



def main():
    rospy.init_node('cleanup_challenge')

    skip = rospy.get_param('~skip', False)
    robot_name = rospy.get_param('~robot_name')
    room = 'bedroom'

    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        raise ValueError('unknown robot')

    robot = Robot()
    robot_smach_states.util.startup(setup_statemachine, challenge_name="cleanup")


if __name__ == '__main__':
    main()
