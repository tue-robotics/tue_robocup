#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
import os
import time

import numpy as np
import rospy
from robot_skills import Hero
from robot_skills.util import kdl_conversions
from smach import StateMachine, cb_interface, CBState
import pickle

REQUIRED_PERSON_DETECTIONS = 10
PERSONS = []


class LocatePeople(StateMachine):
    def __init__(self, robot, cupboard_id):
        StateMachine.__init__(self, outcomes=['done'])

        @cb_interface(outcomes=['done'])
        def detect_persons(_):
            global PERSONS
            global REQUIRED_PERSON_DETECTIONS

            look_angles = np.linspace(-np.pi / 2, np.pi / 2, 8)  # From -pi/2 to +pi/2 to scan 180 degrees wide
            head_goals = [kdl_conversions.VectorStamped(x=100 * math.cos(angle),
                                                        y=100 * math.sin(angle),
                                                        z=1.5,
                                                        frame_id="/%s/base_link" % robot.robot_name)
                          for angle in look_angles]

            while len(PERSONS) < REQUIRED_PERSON_DETECTIONS:
                for head_goal in head_goals:
                    robot.head.look_at_point(head_goal)
                    robot.head.wait_for_motion_done()
                    now = time.time()
                    rgb, depth, depth_info = robot.perception.get_rgb_depth_caminfo()

                    try:
                        PERSONS += [{
                            "rgb": rgb,
                            "persons": robot.perception.detect_person_3d(rgb, depth, depth_info)
                        }]
                    except Exception as e:
                        rospy.logerr(e)
                        rospy.sleep(2.0)
                    rospy.loginfo("Took %.2f", time.time() - now)

            rospy.loginfo("Detected %d persons", REQUIRED_PERSON_DETECTIONS)

            return 'done'

        @cb_interface(outcomes=['done'])
        def _data_association_persons_and_show_image_on_screen(_):
            global PERSONS
            file_name = "/tmp/find_my_mate.pickle"
            with open(file_name, 'w') as f:
                pickle.dump(PERSONS, f)
            rospy.loginfo("Dumped to %s", file_name)
            return 'done'

        with self:
            self.add_auto('DETECT_PERSONS', CBState(detect_persons), ['done'])
            self.add('DATA_ASSOCIATION_AND_SHOW_IMAGE_ON_SCREEN',
                     CBState(_data_association_persons_and_show_image_on_screen), transitions={'done': 'done'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    LocatePeople(hero, 'living_room').execute()
