#!/usr/bin/python

import rospy
import smach
import random

import sys

import hmi

import robot_smach_states
import robot_smach_states.util.designators as ds
from clean_inspect import CleanInspect

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
    # Look for trash units; can be in living_room and kitchen.
    # THIS IS ARENA DEPENDANT!! (Should be handled in a different way?)
    # There should be an 'underscore_rule' : trash_bin or trashbin???
    #   (Different between rgo2019 and robotics_testlab knowledge)

        ids = [e.id for e in self._robot.ed.get_entities()]
        for loc in challenge_knowledge.cleaning_locations:
            # if loc["room"] == "living_room":
            #     if "trash_bin" not in ids:
            #         return "failed"
            if loc["room"] == "kitchen":
                if "trashbin" not in ids:
                    return "failed"

    # Make sure the world model and local knowledge match
        for place in challenge_knowledge.cleaning_locations:
            if place["name"] not in ids:
                return "failed"
            if place["room"] not in ids:
                return "failed"

        return "done"

class AskWhichRoomToClean(smach.State):
# Logic in this code is still flawed. No correct repetition.....
# EXAMINE challenge_restaurant->take_orders.py for information about structure if interaction

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
        # Show the cleanup list on screen
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

                # try:
                #     self.robot.speech.speak("I understand that the {} should be cleaned, "
                #                             .format(speech_result.sentence))
                #     speech_result = self.robot.hmi.query(description="Is this correct?",
                #                                          grammar="T[True] -> yes;"
                #                                                  "T[False] -> no",
                #                                          target="T")
                # except hmi.TimeoutException:
                #     return "failed"
            try:
                # Now: confirm
                self.robot.speech.speak("I understood that the {} should be cleaned,  "
                                             "is this correct?".format(speech_result.sentence))
            except:
                continue

            # try:
            #     speech_result = self.robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;"
            #                                                                                 "T[False] -> no",
            #                                                                          target="T")
            # except hmi.TimeoutException:
            #     return "failed"

            self.robot.head.cancel_goal()
            self.robot.speech.speak("Ok, I will clean the {}".format(self.roomw.resolve()), block=False)
            self.collect_cleanup_locations()


            return "done"

        nr_of_tries += 1

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    robot.ed.reset()
    # The probability number must be determined experimentaly
    robot.ed.set_unknown_probability(0.3)
    # Designators
    # Room to search through
    roomr = ds.VariableDesignator('kitchen', resolve_type=str)
    roomw =roomr.writeable
    # Answer given by operator
    answerr = ds.VariableDesignator('yes', resolve_type=str)
    answerw = ds.VariableWriter(answerr)

    # Cleanup location as defined in local knowledge
    cleanup_locationsr = ds.VariableDesignator([{'1':'2', '3':'4'}])
    cleanup_locationsw = cleanup_locationsr.writeable
    location_des = ds.VariableDesignator(resolve_type=dict)

    with sm:
        # Somehow, the next commented states do not work properly.....
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
                           transitions={"done": "SAY_START_CHALLENGE", "failed": "SAY_KNOWLEDGE_NOT_COMPLETE"})

        smach.StateMachine.add('SAY_KNOWLEDGE_NOT_COMPLETE',
                           robot_smach_states.Say(robot, ["My knowledge of the world is not complete!",
                                                          "Please give me some more information!"], block=False),
                           transitions={"spoken": "Aborted"})

        smach.StateMachine.add('SAY_START_CHALLENGE',
                               robot_smach_states.Say(robot, ["Starting the cleanup challenge",
                                                              "What a mess here, let's clean this room!",
                                                              "Let's see if I can find some garbage here",
                                                              "All I want to do is clean this mess up!"], block=False),
                              transitions={"spoken": "ITERATE_NEXT_LOC"})

# Here the designator cleanup_locationsr has to be iterated over to visit all locations of the room (see designator_iterator.py)
# How is this to be done?
        smach.StateMachine.add('ITERATE_NEXT_LOC',
                               robot_smach_states.IterateDesignator(cleanup_locationsr, location_des.writeable),
                               transitions={"next": "INSPECT",
                                            "stop_iteration": "RETURN_TO_OPERATOR"})

        smach.StateMachine.add("INSPECT",
                                CleanInspect(robot, location_des),
                                transitions={"done": "ITERATE_NEXT_LOC"})

        smach.StateMachine.add("RETURN_TO_OPERATOR",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=ds.EntityByIdDesignator(robot=robot,
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

    # For testing purposes, pick a default room
    room = 'bedroom'

    robot_smach_states.util.startup(setup_statemachine, challenge_name="cleanup")


if __name__ == '__main__':
    main()
