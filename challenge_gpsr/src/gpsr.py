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
        output += message + ". "
    if not task_result.succeeded:
        output += ". I am truly sorry, let's try this again!"
    return output

def main():
    rospy.init_node("gpsr")
    random.seed()

    skip        = rospy.get_param('~skip', False)
    robot_name  = rospy.get_param('~robot_name')
    entrance_no = rospy.get_param('~entrance_number', 0)
    no_of_tasks = rospy.get_param('~number_of_tasks', 0)

    rospy.loginfo("[GPSR] Parameters:")
    rospy.loginfo("[GPSR] robot_name = {}".format(robot_name))
    if skip:
        rospy.loginfo("[GPSR] skip = {}".format(skip))
    if entrance_no not in [1, 2]:
        rospy.logerr("[GPSR] entrance_number should be 1 or 2. You set it to {}".format(entrance_no))
    else:
        rospy.loginfo("[GPSR] entrance_number = {}".format(entrance_no))
        entrance_no -= 1  # to transform to a 0-based index
    if no_of_tasks:
        rospy.loginfo("[GPSR] number_of_tasks = {}".format(no_of_tasks))

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

    if not skip:

        # Wait for door, enter arena
        s = StartChallengeRobust(robot, knowledge.initial_pose[entrance_no])
        s.execute()

        # Move to the start location
        robot.speech.speak("Let's see if my operator has a task for me! Moving to the meeting point.", block=False)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    finished = False

    while True:
        # Navigate to the GPSR meeting point
        if not skip and not finished:
            nwc = NavigateToWaypoint(robot=robot,
                                     waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                              id=knowledge.starting_pose[entrance_no]),
                                     radius=0.3)
            nwc.execute()
            robot.speech.speak(report, block=True)

        # Report to the user and ask for a new task
        robot.speech.speak(report, block=True)
        robot.speech.speak(user_instruction, block=True)

        # Listen for the new task
        try:
            sentence, semantics = robot.hmi.query(description="",
                                                  grammar=knowledge.grammar,
                                                  target=knowledge.grammar_target)
        except hmi.TimeoutException:
            robot.speech.speak(random.sample(knowledge.not_understood_sentences, 1))
            continue

        # TODO: Verify task!

        # Dump the output json object to a string
        task_specification = json.dumps(semantics)

        # Send the task specification to the action server
        task_result = action_client.send_task(task_specification)

        # Write a report to bring to the operator
        report = task_result_to_report(task_result)

        # Keep track of the number of performed tasks
        no_of_tasks_performed += 1
        if no_of_tasks_performed == no_of_tasks:
            finished = True

        # If we succeeded, we can say something optimistic after reporting to the operator
        if task_result.succeeded:
            report += " I performed {} tasks so far, still going strong!".format(no_of_tasks_performed)

        if finished and not skip:
            nwc = NavigateToWaypoint(robot=robot,
                                     waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                              id=knowledge.exit_waypoint[entrance_no]),
                                     radius = 0.3)
            nwc.execute()
            robot.speech.speak("Thank you very much, and goodbye!", block=True)
            break


# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
