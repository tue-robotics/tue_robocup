#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Matthijs van der Burgh, 2017
# ------------------------------------------------------------------------------------------------------------------------

import sys
import rospy
import random
import json

import hmi

from action_server import Client as ActionClient

from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states import StartChallengeRobust, WaitForTrigger
from robot_smach_states.util.designators import EntityByIdDesignator

from robot_skills.util.kdl_conversions import FrameStamped
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
    rospy.init_node("demo")
    random.seed()

    skip        = rospy.get_param('~skip', False)
    restart     = rospy.get_param('~restart', False)
    robot_name  = rospy.get_param('~robot_name')
    no_of_tasks = rospy.get_param('~number_of_tasks', 0)
    time_limit  = rospy.get_param('~time_limit', 0)
    if no_of_tasks == 0:
        no_of_tasks = 999

    rospy.loginfo("[DEMO] Parameters:")
    rospy.loginfo("[DEMO] robot_name = {}".format(robot_name))
    if skip:
        rospy.loginfo("[DEMO] skip = {}".format(skip))
    if no_of_tasks:
        rospy.loginfo("[DEMO] number_of_tasks = {}".format(no_of_tasks))
    if restart:
        rospy.loginfo("[DEMO] running a restart")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        raise ValueError('unknown robot')

    robot = Robot()

    action_client = ActionClient(robot.robot_name)

    knowledge = load_knowledge('challenge_demo')

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

    trigger = WaitForTrigger(robot, ["gpsr"], "/" + robot_name + "/trigger")

    while True:
        # Navigate to the GPSR meeting point
        if not skip:
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
        timeout_count = 0

        while True:
            rospy.loginfo("Waiting for trigger")
            trigger.execute()

            base_loc = robot.base.get_location()
            base_pose = base_loc.frame
            print base_pose
            location_id = "starting_point"
            robot.ed.update_entity(id=location_id, frame_stamped=FrameStamped(base_pose, "/map"),
                                         type="waypoint")

            robot.head.look_at_standing_person()

            robot.speech.speak(user_instruction, block=True)
            # Listen for the new task
            while True:
                try:
                    sentence, semantics = robot.hmi.query(description="",
                                                          grammar=knowledge.grammar,
                                                          target=knowledge.grammar_target)
                    timeout_count = 0
                    break
                except hmi.TimeoutException:
                    robot.speech.speak(random.sample(knowledge.not_understood_sentences, 1)[0])
                    if timeout_count >= 3:
                        robot.hmi.restart_dragonfly()
                        timeout_count = 0
                        rospy.logwarn("[GPSR] Dragonfly restart")
                    else:
                        timeout_count += 1
                        rospy.logwarn("[GPSR] Timeout_count: {}".format(timeout_count))

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

        robot.lights.set_color(0, 0, 1)  # be sure lights are blue

        robot.head.look_at_standing_person()
        robot.reset_all_arms(gripper_timeout=0.0)
        robot.torso.reset()

        rospy.loginfo("Driving back to the starting point")
        nwc = NavigateToWaypoint(robot=robot,
                                 waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                          id=location_id),
                                 radius=0.3)
        nwc.execute()


# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
