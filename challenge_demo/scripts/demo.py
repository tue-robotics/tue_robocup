#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Matthijs van der Burgh, 2017
# ------------------------------------------------------------------------------------------------------------------------

from __future__ import print_function

import json
import random
import sys

import rospy
from pykdl_ros import FrameStamped
from action_server import Client as ActionClient

import hmi
from robocup_knowledge import load_knowledge
from robot_skills.get_robot import get_robot
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.util.designators import EntityByIdDesignator
from robot_smach_states.utility import WaitForTrigger
from robot_skills.simulation.sim_mode import is_sim_mode
from create_semantics_picovoice_demo import create_semantics

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

    robot = get_robot(robot_name)

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

    while not rospy.is_shutdown():
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

        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for trigger")
            trigger.execute()

            robot.head.look_at_standing_person()
            robot.speech.speak(user_instruction, block=True)

            base_loc = robot.base.get_location()
            base_pose = base_loc.frame
            print(base_pose)
            location_id = "starting_point"
            robot.ed.update_entity(uuid=location_id, frame_stamped=FrameStamped(base_pose, rospy.Time.now(), "map"),
                                   etype="waypoint")

            # Listen for the new task
            timeout_tries = 3
            for i in range(timeout_tries):
                try:
                    if is_sim_mode():
                        sentence, semantics = robot.hmi.query(description="",
                                                          grammar=knowledge.grammar,
                                                          target=knowledge.grammar_target)
                    else:
                        sentence, semantics = robot.picovoice.get_intent("demo", demo=True)
                        semantics = create_semantics(semantics)
                    break
                except hmi.TimeoutException:
                    robot.speech.speak(random.sample(knowledge.not_understood_sentences, 1)[0])
            else:
                rospy.logwarn("[GPSR] speech timed out {timeout_tries} times, aborting")
                robot.speech.speak("I am sorry but I cannot hear you. Let's not try again.")
                continue

            # check if we have heard this correctly
            robot.speech.speak('I heard %s, is this correct?' % sentence)

            rospy.loginfo("THE SENTENCE: {}".format(sentence))
            rospy.loginfo("THE SEMANTICS: {}".format(semantics))
            try:
                if is_sim_mode():
                    answer = robot.hmi.query('', 'T -> yes | no', 'T')
                else:
                    answer = robot.picovoice.get_intent("yesOrNo")

                if (is_sim_mode() and answer.sentence == "no") or (not is_sim_mode() and "no" in answer.semantics):
                    robot.speech.speak('Sorry')
                    continue
            except hmi.TimeoutException:
                rospy.loginfo("robot did not hear the confirmation, so lets assume its correct")
                break

            break

        # Dump the output json object to a string
        task_specification = json.dumps(semantics)
        task_specification = task_specification.strip() # remove leading and trailing quotes
        rospy.loginfo("Sending task: {}".format(task_specification))

        # Send the task specification to the action server
        task_result = action_client.send_task(task_specification)

        rospy.loginfo(f"{task_result.missing_field}")
        # # Ask for missing information
        # while task_result.missing_field:
        #     request_missing_field(knowledge.task_result.missing_field)
        #     task_result = action_client.send_task(task_specification)

        # Write a report to bring to the operator
        report = task_result_to_report(task_result)

        robot.lights.set_color(0, 0, 1)  # be sure lights are blue

        robot.head.look_at_standing_person()
        robot.reset_all_arms()
        robot.torso.reset()

        rospy.loginfo("Driving back to the starting point")
        nwc = NavigateToWaypoint(robot=robot,
                                 waypoint_designator=EntityByIdDesignator(robot=robot,
                                                                          uuid=location_id),
                                 radius=0.3)
        nwc.execute()


# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
