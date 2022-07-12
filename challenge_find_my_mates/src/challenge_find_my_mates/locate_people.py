#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
import os
import pickle
import time
from collections import deque
from datetime import datetime

import numpy as np
import rospy
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros
import tf2_ros
from geometry_msgs.msg import PointStamped

from challenge_find_my_mates.get_image import get_image
from pykdl_ros import VectorStamped
from robot_smach_states.human_interaction import Say
from smach import StateMachine, cb_interface, CBState

NUM_LOOKS = 2
PERSON_DETECTIONS = []


class LocatePeople(StateMachine):
    def __init__(self, robot, room_id):
        StateMachine.__init__(self, outcomes=['done'])

        @cb_interface(outcomes=['done'])
        def detect_persons(_):
            global PERSON_DETECTIONS
            global NUM_LOOKS

            # with open(os.path.expanduser('~/find_my_mates/floorplan-2022-06-12-17-53-28.pickle'), 'rb') as f:
            #     PERSON_DETECTIONS = pickle.load(f)
            #     rospy.loginfo("Loaded %d persons", len(PERSON_DETECTIONS))
            #     for det in PERSON_DETECTIONS:
            #         det['map_vs'].header.stamp = rospy.Time(0)
            #         det['rgb'].header.stamp = rospy.Time(0)
            # return "done"

            look_angles = np.linspace(-np.pi / 2, np.pi / 2, 8)  # From -pi/2 to +pi/2 to scan 180 degrees wide
            head_goals = [VectorStamped.from_xyz(100 * math.cos(angle), 100 * math.sin(angle), 1.5, rospy.Time.now(),
                                                 robot.base_link_frame)
                          for angle in look_angles]

            sentences = deque([
                "Hi there mates, where are you, please look at me!",
                "I am looking for my mates! Dippi dee doo! Pew pew!",
                "You are all looking great today! Keep looking at my camera. I like it when everybody is staring at me!"
            ])
            while len(PERSON_DETECTIONS) < 4 and not rospy.is_shutdown():
                for _ in range(NUM_LOOKS):
                    sentences.rotate(1)
                    robot.speech.speak(sentences[0], block=False)
                    for head_goal in head_goals:
                        robot.speech.speak("please look at me", block=False)
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
                                    try:
                                        point_stamped = PointStamped(rgb.header, person.position)
                                        vs = tf2_ros.convert(point_stamped, VectorStamped)
                                        PERSON_DETECTIONS.append({
                                            "map_vs": robot.tf_buffer.transform(vs, "map"),
                                            "person_detection": person,
                                            "rgb": rgb
                                        })
                                    except Exception as e:
                                        rospy.logerr(f"Failed to transform valid person detection to map frame: {e}")

                        rospy.loginfo("Took %.2f, we have %d person detections now", time.time() - now,
                                      len(PERSON_DETECTIONS))

            rospy.loginfo("Detected %d persons", len(PERSON_DETECTIONS))

            return 'done'

        @cb_interface(outcomes=['done', 'retry', 'failed'])
        def _data_association_persons_and_show_image_on_screen(_):
            global PERSON_DETECTIONS

            try:
                os.makedirs(os.path.expanduser('~/find_my_mates'), exist_ok=True)
                with open(os.path.expanduser(
                    '~/find_my_mates/floorplan-{}.pickle'.format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))),
                    'wb') as f:
                    pickle.dump(PERSON_DETECTIONS, f)
            except Exception:
                pass

            rospy.loginfo('Found %d person detections', len(PERSON_DETECTIONS))

            image_filename = get_image(robot, room_id, PERSON_DETECTIONS)
            robot.hmi.show_image(image_filename, 120)

            return "done"

        with self:
            self.add_auto('DETECT_PERSONS', CBState(detect_persons), ['done'])
            self.add('DATA_ASSOCIATION_AND_SHOW_IMAGE_ON_SCREEN',
                     CBState(_data_association_persons_and_show_image_on_screen),
                     transitions={'done': 'done',
                                  'retry': 'DETECT_PERSONS',
                                  'failed': 'SAY_NOT_ABLE_TO_SHOW'})
            self.add('SAY_NOT_ABLE_TO_SHOW',
                     Say(robot, ["I was not able to generate a map to show you the people I have seen",
                                 "Dammn, I am not able to show you where I have seen your mates"]),
                     transitions={'spoken': 'done'})


if __name__ == '__main__':
    from robocup_knowledge import load_knowledge
    from robot_skills import get_robot

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot = get_robot("hero", 1)
    # robot.reset()
    challenge_knowledge = load_knowledge('challenge_find_my_mates')
    LocatePeople(robot, challenge_knowledge.room).execute()
