#! /usr/bin/python

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

    robot_name  = rospy.get_param('~robot_name', "hero")

    rospy.loginfo("[DEMO] Parameters:")
    rospy.loginfo("[DEMO] robot_name = {}".format(robot_name))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    robot = get_robot(robot_name)

    action_client = ActionClient(robot.robot_name)

    knowledge = load_knowledge('challenge_demo')

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Start

    base_loc = robot.base.get_location()
    base_pose = base_loc.frame
    print(base_pose)
    location_id = "starting_point"
    robot.ed.update_entity(uuid=location_id, frame_stamped=FrameStamped(base_pose, rospy.Time.now(), "map"),
                           etype="waypoint")

    while True:
        #TODO hardcoded starting point
        #TODO get rid of string format
        task_specification = "{actions: [{'action': 'hand-over', 'source-location': {'id': 'dinner_table'}, 'target-location': {'id': 'operator', 'designator': EntityByIdDesignator(uuid=starting_point, name=None)}, 'object': {'type': 'coke'}}]}"
        # Dump the output json object to a string
        #task_specification = json.dumps(semantics)
        #task_specification = task_specification.strip() # remove leading and trailing quotes
        rospy.loginfo("Sending task: {}".format(task_specification))

        # Send the task specification to the action server
        task_result = action_client.send_task(task_specification)

        report = task_result_to_report(task_result)
        rospy.loginfo(report)

        robot.lights.set_color(0, 0, 1)  # be sure lights are blue

        robot.head.look_at_standing_person()
        robot.reset_all_arms()
        robot.torso.reset()


# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
