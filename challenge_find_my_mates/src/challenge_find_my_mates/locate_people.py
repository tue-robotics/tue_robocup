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

import tf2_ros
from geometry_msgs.msg import PointStamped
from pykdl_ros import VectorStamped
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros
import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from smach import StateMachine, cb_interface, CBState

from image_recognition_util.image_writer import color_map

from ed.entity import Entity
from challenge_find_my_mates.cluster import cluster_people

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

                        rospy.loginfo("Took %.2f, we have %d person detections now", time.time() - now, len(PERSON_DETECTIONS))

            rospy.loginfo("Detected %d persons", len(PERSON_DETECTIONS))

            return 'done'

        @cb_interface(outcomes=['done', 'failed'])
        def _data_association_persons_and_show_image_on_screen(_):
            global PERSON_DETECTIONS

            try:
                os.makedirs(os.path.expanduser('~/find_my_mates'), exist_ok=True)
                with open(os.path.expanduser('~/find_my_mates/floorplan-{}.pickle'.format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))), 'wb') as f:
                    pickle.dump(PERSON_DETECTIONS, f)
            except Exception:
                pass

            rospy.loginfo('Found %d person detections', len(PERSON_DETECTIONS))

            room_entity = robot.ed.get_entity(uuid=room_id)  # type: Entity
            if room_entity is None:
                msg = f"Could not find room: '{room_id}"
                rospy.logerr(msg)
                raise ValueError(msg)

            padding = -0.3

            def _get_clusters():
                in_room_detections = [d for d in PERSON_DETECTIONS if room_entity.in_volume(d['map_vs'], "in", padding)]

                rospy.loginfo("%d in room before clustering", len(in_room_detections))

                center_point = room_entity.pose.frame.p
                clusters = cluster_people(in_room_detections, np.array([center_point.x, center_point.y]))

                return clusters

            # filter in room and perform clustering until we have 4 options
            try:
                person_detection_clusters = _get_clusters()
            except ValueError as e:
                rospy.logerr(e)
                robot.speech.speak("Mates, where are you?", block=False)
                return "failed"

            floorplan, img_pose, pixels_per_meter_width, pixels_per_meter_height = robot.ed.get_map([room_id])
            floorplan_height, floorplan_width, _ = floorplan.shape

            bridge = CvBridge()
            c_map = color_map(n=len(person_detection_clusters), normalized=True)
            for i, person_detection in enumerate(person_detection_clusters):
                image = bridge.imgmsg_to_cv2(person_detection['rgb'], "bgr8")
                roi = person_detection['person_detection'].face.roi
                roi_image = image[roi.y_offset:roi.y_offset + roi.height, roi.x_offset:roi.x_offset + roi.width]

                desired_height = 150
                height, width, channel = roi_image.shape
                ratio = float(height) / float(desired_height)
                calculated_width = int(float(width) / ratio)
                resized_roi_image = cv2.resize(roi_image, (calculated_width, desired_height))

                vs_image_frame = img_pose.frame.Inverse() * person_detection['map_vs'].vector

                px = int(vs_image_frame.x() * pixels_per_meter_width)
                py = int(vs_image_frame.y() * pixels_per_meter_height)

                cv2.circle(floorplan, (px, py), 3, (0, 0, 255), 5)

                try:
                    px_image = min(max(0, int(px - calculated_width / 2)), floorplan_width - calculated_width - 1)
                    py_image = min(max(0, int(py - desired_height / 2)), floorplan_height - desired_height - 1)

                    if px_image >= 0 and py_image >= 0:
                        # Could not broadcast input array from shape (150, 150, 3) into shape (106, 150, 3)
                        floorplan[py_image:py_image + desired_height, px_image:px_image + calculated_width] = resized_roi_image
                        cv2.rectangle(floorplan, (px_image, py_image),
                                      (px_image + calculated_width, py_image + desired_height),
                                      (c_map[i, 2] * 255, c_map[i, 1] * 255, c_map[i, 0] * 255), 10)
                    else:
                        rospy.logerr("bound error")
                except Exception as e:
                    rospy.logerr("Drawing image roi failed: {}".format(e))

                label = "female" if person_detection['person_detection'].gender else "male"
                label += ", " + str(person_detection['person_detection'].age)
                cv2.putText(floorplan, label, (px_image, py_image + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

                # cv2.circle(floorplan, (px, py), 3, (0, 0, 255), 5)

            filename = os.path.expanduser('~/floorplan-{}.png'.format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S")))
            cv2.imwrite(filename, floorplan)
            robot.hmi.show_image(filename, 120)

            return "done"

        with self:
            self.add_auto('DETECT_PERSONS', CBState(detect_persons), ['done'])
            self.add('DATA_ASSOCIATION_AND_SHOW_IMAGE_ON_SCREEN',
                     CBState(_data_association_persons_and_show_image_on_screen), transitions={'done': 'done', 'failed': 'DETECT_PERSONS'})


if __name__ == '__main__':
    from robocup_knowledge import load_knowledge
    from robot_skills import get_robot

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot = get_robot("hero", 0)
    # robot.reset()
    challenge_knowledge = load_knowledge('challenge_find_my_mates')
    LocatePeople(robot, challenge_knowledge.room).execute()
