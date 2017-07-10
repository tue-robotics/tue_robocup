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

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        raise ValueError('unknown robot')

    robot = Robot()

    action_client = ActionClient(robot.robot_name)

    knowledge = load_knowledge('challenge_gpsr')

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

    while True:
        # Navigate to the GPSR meeting point
        if not skip and not finished:
            robot.speech.speak("Moving to the meeting point.", block=False)
            nwc = NavigateToWaypoint(robot=robot,
                                     waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                              id=knowledge.starting_pose),
                                     radius=0.3)
            nwc.execute()
            # Report to the user and ask for a new task

        # Report to the user
        robot.head.look_at_standing_person()
        robot.speech.speak(report, block=True)

        while True:
            while True and not test:
                try:
                    robot.hmi.query(description="", grammar="T -> %s" % robot_name, target="T")
                except hmi.TimeoutException:
                    continue
                else:
                    break

            robot.speech.speak(user_instruction, block=True)
            # Listen for the new task
            while True:
                try:
                    sentence, semantics = robot.hmi.query(description="",
                                                          grammar=knowledge.grammar,
                                                          target=knowledge.grammar_target)
                    break
                except hmi.TimeoutException:
                    robot.speech.speak(random.sample(knowledge.not_understood_sentences, 1)[0])
                    continue

            if not test:
                # check if we have heard this correctly
                robot.speech.speak('I heard %s, is this correct?' % sentence)
                try:
                    if 'no' == robot.hmi.query('', 'T -> yes | no', 'T').sentence:
                        robot.speech.speak('Sorry')
                        continue
                except hmi.TimeoutException:
                    # robot did not hear the confirmation, so lets assume its correct
                    break

            break

        # Dump the output json object to a string
        task_specification = json.dumps(semantics)

        # Send the task specification to the action server
        task_result = action_client.send_task(task_specification)

        print task_result.missing_field
        # # Ask for missing information
        # while task_result.missing_field:
        #     request_missing_field(knowledge.task_result.missing_field)
        #     task_result = action_client.send_task(task_specification)

        # Write a report to bring to the operator
        report = task_result_to_report(task_result)

        robot.lights.set_color(0,0,1)  #be sure lights are blue

        robot.head.look_at_standing_person()
        robot.leftArm.reset()
        robot.leftArm.send_gripper_goal('close',0.0)
        robot.rightArm.reset()
        robot.rightArm.send_gripper_goal('close',0.0)
        robot.torso.reset()

        if task_result.succeeded:
            # Keep track of the number of performed tasks
            no_of_tasks_performed += 1
            if no_of_tasks_performed == no_of_tasks:
                finished = True

            # If we succeeded, we can say something optimistic after reporting to the operator
            if no_of_tasks_performed == 1:
                task_word = "task"
            else:
                task_word = "tasks"
            report += " I performed {} {} so far, still going strong!".format(no_of_tasks_performed, task_word)

        if rospy.get_time() - start_time > 60 * 15:
            finished = True

        if finished and not skip:
            # check if indeed the challenge is over
            robot.speech.speak('According to me, this challenge is over, is this correct?')
            try:
                if 'no' == robot.hmi.query('', 'T -> yes | no', 'T').sentence:
                    # Do something smart when the answer is no, so you dont keep asking
                    robot.speech.speak('Sorry, I will go on as long as you need me')
                    continue
                else:
                    nwc = NavigateToWaypoint(robot=robot,
                                             waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                                      id=knowledge.exit_waypoint),
                                             radius = 0.3)
                    robot.speech.speak("Thank you very much, and goodbye!", block=True)
                    nwc.execute()
                    break
            except hmi.TimeoutException:
                # robot did not hear the confirmation, so lets go on
                continue


# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
