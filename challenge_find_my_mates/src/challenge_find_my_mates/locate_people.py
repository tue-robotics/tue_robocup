#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
import os
import pickle
import time

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from robot_skills import Hero
from robot_skills.util import kdl_conversions
from smach import StateMachine, cb_interface, CBState

NUM_LOOKS = 3
PERSON_DETECTIONS = []


class LocatePeople(StateMachine):
    def __init__(self, robot, room_id):
        StateMachine.__init__(self, outcomes=['done'])

        @cb_interface(outcomes=['done'])
        def detect_persons(_):
            global PERSON_DETECTIONS
            global NUM_LOOKS

            # with open('/home/rein/find_my_mate.pickle') as f:
            #     PERSON_DETECTIONS = pickle.load(f)
            #
            # return "done"

            look_angles = np.linspace(-np.pi / 2, np.pi / 2, 8)  # From -pi/2 to +pi/2 to scan 180 degrees wide
            head_goals = [kdl_conversions.VectorStamped(x=100 * math.cos(angle),
                                                        y=100 * math.sin(angle),
                                                        z=1.5,
                                                        frame_id="/%s/base_link" % robot.robot_name)
                          for angle in look_angles]

            for _ in range(NUM_LOOKS):
                for head_goal in head_goals:
                    robot.head.look_at_point(head_goal)
                    robot.head.wait_for_motion_done()
                    now = time.time()
                    rgb, depth, depth_info = robot.perception.get_rgb_depth_caminfo()

                    try:
                        persons = robot.perception.detect_person_3d(rgb, depth, depth_info)
                    except Exception as e:
                        rospy.logerr(e)
                        rospy.sleep(2.0)
                    else:
                        for person in persons:
                            if person.face.roi.width > 0 and person.face.roi.height > 0:
                                PERSON_DETECTIONS.append({
                                    "map_ps": robot.tf_listener.transformPoint("map", PointStamped(
                                        header=persons.header,
                                        point=person.position
                                    )),
                                    "person_detection": person,
                                    "rgb": rgb
                                })

                    rospy.loginfo("Took %.2f", time.time() - now)

            rospy.loginfo("Detected %d persons", len(PERSON_DETECTIONS))

            return 'done'

        @cb_interface(outcomes=['done'])
        def _data_association_persons_and_show_image_on_screen(_):
            global PERSON_DETECTIONS

            with open('/tmp/find_my_mate.pickle', 'w') as f:
                pickle.dump(PERSON_DETECTIONS, f)

            return "done"

        with self:
            self.add_auto('DETECT_PERSONS', CBState(detect_persons), ['done'])
            self.add('DATA_ASSOCIATION_AND_SHOW_IMAGE_ON_SCREEN',
                     CBState(_data_association_persons_and_show_image_on_screen), transitions={'done': 'done'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    LocatePeople(hero, 'living_room').execute()
