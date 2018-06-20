#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Rokus Ottervanger, 2017
# ------------------------------------------------------------------------------------------------------------------------

import sys
import rospy
import random
import json

import hmi

from action_server import Client as ActionClient

from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states import StartChallengeRobust
from robot_smach_states.util.designators import EntityByIdDesignator

from robocup_knowledge import load_knowledge

from conversation_engine import ConversationEngine

class ConversationEngineWithHmi(ConversationEngine):
    def __init__(self, robot, grammar, command_target, knowledge):
        super(ConversationEngineWithHmi, self).__init__(robot.robot_name, grammar, command_target)

        self.robot = robot
        self.knowledge = knowledge
        self.timeout_count = 0
        self.test = False
        self.skip = False
        self.give_examples = False
        self.tasks_to_be_done = 999
        self.tasks_done = 0

    def _say_to_user(self, message):
        rospy.loginfo("_say_to_user('{}')".format(message))
        self.robot.speech.speak(message)

    def _on_task_successful(self, message):
        rospy.loginfo("_on_task_successful('{}')".format(message))

        self.tasks_done += 1

        # TODO: check number of tasks, if enough done move to exit else go to meeting point
        # Navigate to the GPSR meeting point
        if not self.skip:
            self.robot.speech.speak("Moving to the meeting point.", block=False)
            nwc = NavigateToWaypoint(robot=self.robot,
                                     waypoint_designator=EntityByIdDesignator(robot=self.robot,
                                                                              id=self.knowledge.starting_pose),
                                     radius=0.3)
            nwc.execute()
            # Report to the user and ask for a new task

        if self.tasks_done >= self.tasks_to_be_done and not self.skip:
            nwc = NavigateToWaypoint(robot=self.robot,
                                     waypoint_designator=EntityByIdDesignator(robot=self.robot,
                                                                              id=self.knowledge.exit_waypoint),
                                     radius=0.3)
            self.robot.speech.speak("I'm done now. Thank you very much, and goodbye!", block=True)
            nwc.execute()
            return

        # Report to the user
        self.robot.head.look_at_standing_person()
        self._say_to_user(message)
        self.timeout_count = 0

        self._start_wait_for_command(self.knowledge.grammar, self.knowledge.grammar_target)

    def _on_request_missing_information(self, description, grammar, target):
        rospy.loginfo("_request_missing_information('{}', '{}...', '{}')".format(description, grammar[:10], target))

        example = self._parser.get_random_sentence(self._state.target)
        description += " For example: '{}'".format(example)
        self._say_to_user(description)

        sentence, semantics = self.robot.hmi.query(description=description,
                                                   grammar=grammar,
                                                   target=target)
        self.user_to_robot_text(sentence)

    def _on_task_outcome_failed(self, message):
        rospy.loginfo("_on_task_outcome_failed('{}')".format(message))
        self._say_to_user(message)

    def _on_task_outcome_unknown(self, message):
        rospy.loginfo("_on_task_outcome_unknown('{}')".format(message))
        self._say_to_user(message)

    def _start_wait_for_command(self, grammar, target):
        rospy.loginfo("_start_wait_for_command()")
        self.robot.speech.speak("Trigger me by saying my name, and wait for the ping.", block=True)

        self.wait_to_be_called()

        self.robot.speech.speak("What can I do for you?", block=True)

        while True:
            try:
                sentence, semantics = self.robot.hmi.query(description="",
                                                      grammar=grammar,
                                                      target=target)
                self.timeout_count = 0
                if not self.test:
                    if self.heard_correct(sentence):
                        # Pass the heard sentence to the conv.engine. This parses it again, but fuck efficiency for now
                        self.user_to_robot_text(sentence)
                        break
            except hmi.TimeoutException:
                rospy.logwarn("HMI timed out when getting command")
                self.robot.speech.speak(random.sample(self.knowledge.not_understood_sentences, 1)[0])
                if self.timeout_count >= 3:
                    self.robot.hmi.restart_dragonfly()
                    self.timeout_count = 0
                    rospy.logwarn("[GPSR] Dragonfly restart")
                else:
                    self.timeout_count += 1
                    rospy.logwarn("[GPSR] Timeout_count: {}".format(self.timeout_count))

    def wait_to_be_called(self):
        while True and not self.test:
            try:
                self.robot.hmi.query(description="", grammar="T -> %s" % self.robot.robot_name, target="T")
                self.timeout_count = 0
                break
            except hmi.TimeoutException:
                rospy.logwarn("HMI timed out when waiting for name")
                if self.timeout_count >= 3:
                    self.robot.hmi.restart_dragonfly()
                    self.timeout_count = 0
                    rospy.logwarn("[GPSR] Dragonfly restart")
                else:
                    self.timeout_count += 1
                    rospy.logwarn("[GPSR] Timeout_count: {}".format(self.timeout_count))

    def heard_correct(self, sentence):
        self.robot.speech.speak('I heard %s, is this correct?' % sentence)
        try:
            if 'no' == self.robot.hmi.query('', 'T -> yes | no', 'T').sentence:
                self.robot.speech.speak('Sorry')
                return False
            else:
                rospy.loginfo("'{}' was correct".format(sentence))
                return True
        except hmi.TimeoutException:
            rospy.logwarn("HMI timed out when getting confirmation")
            return True


def task_result_to_report(task_result):
    output = ""
    for message in task_result.messages:
        output += message
    # if not task_result.succeeded:
    #     output += " I am truly sorry, let's try this again! "
    return output


def request_missing_field(grammar, grammar_target, semantics, missing_field):
    return semantics


def main():
    rospy.init_node("gpsr")
    random.seed()

    skip        = rospy.get_param('~skip', False)
    restart     = rospy.get_param('~restart', False)
    robot_name  = rospy.get_param('~robot_name')
    no_of_tasks = rospy.get_param('~number_of_tasks', 0)
    test        = rospy.get_param('~test_mode', False)
    eegpsr      = rospy.get_param('~eegpsr', False)
    time_limit  = rospy.get_param('~time_limit', 0)
    if no_of_tasks == 0:
        no_of_tasks = 999

    if time_limit == 0:
        time_limit = 999

    rospy.loginfo("[GPSR] Parameters:")
    rospy.loginfo("[GPSR] robot_name = {}".format(robot_name))
    if skip:
        rospy.loginfo("[GPSR] skip = {}".format(skip))
    if no_of_tasks:
        rospy.loginfo("[GPSR] number_of_tasks = {}".format(no_of_tasks))
    if restart:
        rospy.loginfo("[GPSR] running a restart")
    if test:
        rospy.loginfo("[GPSR] running in test mode")
    rospy.loginfo("[GPSR] time_limit = {}".format(time_limit))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        raise ValueError('unknown robot')

    robot = Robot()

    if eegpsr:
        knowledge = load_knowledge('challenge_eegpsr')
    else:
        knowledge = load_knowledge('challenge_gpsr')

    conversation_engine = ConversationEngineWithHmi(robot, knowledge.grammar, knowledge.grammar_target, knowledge)
    conversation_engine.test = test
    conversation_engine.skip = skip
    conversation_engine.tasks_to_be_done = no_of_tasks

    no_of_tasks_performed = 0

    user_instruction = "What can I do for you?"
    report = ""


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Start

    if not skip and not restart:

        # Wait for door, enter arena
        s = StartChallengeRobust(robot, knowledge.initial_pose)
        s.execute()

        # Move to the start location
        robot.speech.speak("Let's see if my operator has a task for me!", block=False)


    if restart:
        robot.speech.speak("Performing a restart. So sorry about that last time!", block=False)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    finished = False
    start_time = rospy.get_time()

    # while True:
        # # Navigate to the GPSR meeting point
        # if not skip:
        #     robot.speech.speak("Moving to the meeting point.", block=False)
        #     nwc = NavigateToWaypoint(robot=robot,
        #                              waypoint_designator=EntityByIdDesignator(robot=robot,
        #                                                                       id=knowledge.starting_pose),
        #                              radius=0.3)
        #     nwc.execute()
        #     # Report to the user and ask for a new task
        #
        # # Report to the user
        # robot.head.look_at_standing_person()
        # robot.speech.speak(report, block=True)
        # timeout_count = 0

        # if finished and not skip:
        #     nwc = NavigateToWaypoint(robot=robot,
        #                              waypoint_designator=EntityByIdDesignator(robot=robot,
        #                                                                       id=knowledge.exit_waypoint),
        #                              radius=0.3)
        #     robot.speech.speak("I'm done now. Thank you very much, and goodbye!", block=True)
        #     nwc.execute()
        #     break
    # import ipdb; ipdb.set_trace()
    conversation_engine._start_wait_for_command(knowledge.grammar, knowledge.grammar_target)
    rospy.spin()

        # # Dump the output json object to a string
        # task_specification = json.dumps(semantics)
        #
        # # Send the task specification to the action server
        # task_result = action_client.send_task(task_specification)
        #
        # print task_result.missing_field
        # # # Ask for missing information
        # # while task_result.missing_field:
        # #     request_missing_field(knowledge.task_result.missing_field)
        # #     task_result = action_client.send_task(task_specification)
        #
        # # Write a report to bring to the operator
        # report = task_result_to_report(task_result)
        #
        # robot.lights.set_color(0,0,1)  #be sure lights are blue
        #
        # robot.head.look_at_standing_person()
        # robot.leftArm.reset()
        # robot.leftArm.send_gripper_goal('close',0.0)
        # robot.rightArm.reset()
        # robot.rightArm.send_gripper_goal('close',0.0)
        # robot.torso.reset()
        #
        # if task_result.succeeded:
        #     # Keep track of the number of performed tasks
        #     no_of_tasks_performed += 1
        #     if no_of_tasks_performed >= no_of_tasks:
        #         finished = True
        #
        #     # If we succeeded, we can say something optimistic after reporting to the operator
        #     if no_of_tasks_performed == 1:
        #         task_word = "task"
        #     else:
        #         task_word = "tasks"
        #     report += " I performed {} {} so far, still going strong!".format(no_of_tasks_performed, task_word)
        #
        # if rospy.get_time() - start_time > (60 * time_limit - 45) and no_of_tasks_performed >= 1:
        #     finished = True


# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
