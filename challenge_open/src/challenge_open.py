#!/usr/bin/python
import sys

import robot_smach_states
import rospy
import smach
import random
from hmi import TimeoutException
from robocup_knowledge import load_knowledge
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, EntityByIdDesignator

from clean_inspect import CleanInspect

challenge_knowledge = load_knowledge('challenge_open')


class VerifyWorldModelInfo(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["failed", "done"])
        self._robot = robot

    def execute(self, userdata):

        ids = [e.id for e in self._robot.ed.get_entities()]
        if "trashbin" not in ids:
            rospy.logwarn("trashbin not in world model")
            return "failed"

        for place in challenge_knowledge.inspection_places:
            if place["entity_id"] not in ids:
                rospy.logwarn("%s not in world model", place["entity_id"])
                return "failed"
            if place["room_id"] not in ids:
                rospy.logwarn("%s not in world model", place["room_id"])
                return "failed"
            if challenge_knowledge.ask_waypoint not in ids:
                rospy.logwarn("%s not in world model", challenge_knowledge.ask_waypoint)
                return "failed"

        return "done"


class DetermineWhatToCleanInspect(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=[place["entity_id"] for place in challenge_knowledge.inspection_places])
        self._robot = robot
        self._served = []

    def execute(self, userdata):
        while not rospy.is_shutdown():
            self._robot.head.reset()
            rospy.sleep(rospy.Duration(1.0))

            best_p = 0
            best_roi = None
            best_label = None
            for face in self._robot.head.detect_persons():
                for p in face.categorical_distribution.probabilities:
                    if p.label in self._served:
                        continue
                    if p.probability > best_p:
                        best_p = p.probability
                        best_roi = face.roi
                        best_label = p.label

            if best_label:
                rospy.loginfo('best face is the face of %s with p=%f and roi=%s', best_label, best_p, best_roi)
                self._served.append(best_label)

                rospy.loginfo('best_roi: %s', best_roi)
                face_location = self._robot.head.project_roi(best_roi, frame_id='/' + self._robot.robot_name + '/base_link')
                rospy.loginfo('face_location: %s', face_location)
                rospy.loginfo('looking at face at %s', face_location)
                self._robot.head.look_at_point(face_location)

                self._robot.speech.speak('Hello %s' % best_label)
            else:
                self._robot.speech.speak('Can somebody stand in front of me?')
                self._robot.head.look_at_standing_person()

            sentence = random.choice([
                "What should I clean?",
                "Where should I look for trash?",
                "Tell me where I can find the mess"
            ])
            self._robot.speech.speak(sentence, block=True)
            try:
                response = self._robot.hmi.query(sentence, challenge_knowledge.grammar,
                                                 challenge_knowledge.grammar_target)
            except TimeoutException as e:
                self._robot.speech.speak(random.choice(["I did not hear you?", "What did you say?", "Please repeat"]))
                rospy.sleep(1)
            else:
                for place in challenge_knowledge.inspection_places:
                    if response.semantics == place["entity_id"]:
                        self._robot.speech.speak("Let's clean the {}".format(response.sentence), block=False)
                        return place["entity_id"]
                else:
                    self._robot.speech.speak("I dont know what you mean with {}".format(response.sentence))


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    robot.ed.reset()

    with sm:
        smach.StateMachine.add("INITIALIZE", robot_smach_states.Initialize(robot),
                               transitions={"initialized": "SAY_WAITING_FOR_TRIGGER", "abort": "Aborted"})

        # Start challenge via StartChallengeRobust, skipped atm
        smach.StateMachine.add("START_CHALLENGE_ROBUST",
                               robot_smach_states.StartChallengeRobust(robot, challenge_knowledge.starting_point,
                                                                       door=False),
                               transitions={"Done": "SAY_WAITING_FOR_TRIGGER",
                                            "Failed": "Aborted",
                                            "Aborted": "Aborted"})

        smach.StateMachine.add('SAY_WAITING_FOR_TRIGGER',
                               robot_smach_states.Say(robot, ["Trigger me if you need me!",
                                                              "Waiting for trigger",
                                                              "Waiting for you to call me!"], block=False),
                               transitions={"spoken": "WAIT_FOR_TRIGGER"})

        smach.StateMachine.add('WAIT_FOR_TRIGGER',
                               robot_smach_states.WaitForTrigger(robot, ["gpsr"], "/amigo/trigger"),
                               transitions={"gpsr": "VERIFY", "preempted": "VERIFY"})

        smach.StateMachine.add('VERIFY',
                               VerifyWorldModelInfo(robot),
                               transitions={"done": "SAY_START_CHALLENGE", "failed": "SAY_KNOWLEDGE_NOT_COMPLETE"})

        smach.StateMachine.add('SAY_KNOWLEDGE_NOT_COMPLETE',
                               robot_smach_states.Say(robot, ["My knowledge of the world is not complete!",
                                                              "Please give me some more information!"], block=False),
                               transitions={"spoken": "SAY_WAITING_FOR_TRIGGER"})

        smach.StateMachine.add('SAY_START_CHALLENGE',
                               robot_smach_states.Say(robot, ["What a mess here, let's clean this room!",
                                                              "Let's see if I can find some garbage here",
                                                              "All I want to do is clean this mess up!"], block=False),
                               transitions={"spoken": "NAVIGATE_TO_ASK_WAYPOINT"})

        smach.StateMachine.add("NAVIGATE_TO_ASK_WAYPOINT",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(
                                                                         robot=robot,
                                                                         id=challenge_knowledge.ask_waypoint),
                                                                     radius=0.3),
                               transitions={'arrived': 'DETERMINE_WHAT_TO_CLEAN_INSPECT',
                                            'unreachable': 'DETERMINE_WHAT_TO_CLEAN_INSPECT',
                                            'goal_not_defined': 'DETERMINE_WHAT_TO_CLEAN_INSPECT'})

        smach.StateMachine.add("DETERMINE_WHAT_TO_CLEAN_INSPECT",
                               DetermineWhatToCleanInspect(robot),
                               transitions={place["entity_id"]: "CLEAN_INSPECT_%s" % place["entity_id"] for place in
                                            challenge_knowledge.inspection_places})

        for place in challenge_knowledge.inspection_places:
            smach.StateMachine.add("CLEAN_INSPECT_%s" % place["entity_id"],
                                   CleanInspect(robot, place["entity_id"], place["room_id"], place["navigate_area"],
                                                place["segment_areas"]),
                                   transitions={"done": "NAVIGATE_TO_ASK_WAYPOINT"})
    return sm


if __name__ == '__main__':
    rospy.init_node('challenge_open')

    # Check if we have something specified to inspect
    if not challenge_knowledge.inspection_places:
        rospy.logerr("The challenge knowledge inspection_places list should contain at least one entry!")
        sys.exit(1)

    robot_smach_states.util.startup(setup_statemachine, challenge_name="challenge_open")
