#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
import os
import pickle
import random
import time
from collections import deque
from datetime import datetime

import PyKDL
import cv2
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from robot_skills import Hero
from robot_skills.util import kdl_conversions
from smach import StateMachine, cb_interface, CBState
from challenge_find_my_mates.cluster import cluster_people

NUM_LOOKS = 2
PERSON_DETECTIONS = []


def color_map(N=256, normalized=False):
    """
    Generate an RGB color map of N different colors

    :param N : int amount of colors to generate
    :param normalized: bool indicating range of each channel: float32 in [0, 1] or int in [0, 255]
    :return: a numpy.array of shape (N, 3) with a row for each color and each row is [R,G,B]
    """

    def bitget(byteval, idx):
        return ((byteval & (1 << idx)) != 0)

    dtype = 'float32' if normalized else 'uint8'
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i + 1  # skip the first color (black)
        for j in range(8):
            r |= bitget(c, 0) << 7 - j
            g |= bitget(c, 1) << 7 - j
            b |= bitget(c, 2) << 7 - j
            c >>= 3

        cmap[i] = np.array([r, g, b])

    cmap = cmap / 255 if normalized else cmap
    return cmap


class LocatePeople(StateMachine):
    def __init__(self, robot, room_id):
        StateMachine.__init__(self, outcomes=['done'])

        @cb_interface(outcomes=['done'])
        def detect_persons(_):
            global PERSON_DETECTIONS
            global NUM_LOOKS

            # with open('/home/rein/mates/floorplan-2019-07-05-11-06-52.pickle', 'r') as f:
            #     PERSON_DETECTIONS = pickle.load(f)
            #     rospy.loginfo("Loaded %d persons", len(PERSON_DETECTIONS))
            #
            #
            # return "done"

            look_angles = np.linspace(-np.pi / 2, np.pi / 2, 8)  # From -pi/2 to +pi/2 to scan 180 degrees wide
            head_goals = [kdl_conversions.VectorStamped(x=100 * math.cos(angle),
                                                        y=100 * math.sin(angle),
                                                        z=1.5,
                                                        frame_id="/%s/base_link" % robot.robot_name)
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
                                        PERSON_DETECTIONS.append({
                                            "map_ps": robot.tf_listener.transformPoint("map", PointStamped(
                                                header=rgb.header,
                                                point=person.position
                                            )),
                                            "person_detection": person,
                                            "rgb": rgb
                                        })
                                    except Exception as e:
                                        rospy.logerr("Failed to transform valid person detection to map frame")

                        rospy.loginfo("Took %.2f, we have %d person detections now", time.time() - now, len(PERSON_DETECTIONS))

            rospy.loginfo("Detected %d persons", len(PERSON_DETECTIONS))

            return 'done'

        @cb_interface(outcomes=['done', 'failed'])
        def _data_association_persons_and_show_image_on_screen(_):
            global PERSON_DETECTIONS

            try:
                with open(os.path.expanduser('~/floorplan-{}.pickle'.format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))), 'w') as f:
                    pickle.dump(PERSON_DETECTIONS, f)
            except:
                pass

            room_entity = robot.ed.get_entity(id=room_id)  # type: Entity
            room_volume = room_entity.volumes["in"]
            min_corner = room_entity.pose.frame * room_volume.min_corner
            max_corner = room_entity.pose.frame * room_volume.max_corner

            shrink_x = 0.5
            shrink_y = 0.3
            min_corner_shrinked = PyKDL.Vector(min_corner.x() + shrink_x, min_corner.y() + shrink_y, 0)
            max_corner_shrinked = PyKDL.Vector(max_corner.x() - shrink_x, max_corner.y() - shrink_y, 0)

            rospy.loginfo('Found %d person detections', len(PERSON_DETECTIONS))

            def _get_clusters():
                def _in_room(p):
                    return min_corner_shrinked.x() < p.x < max_corner_shrinked.x() and min_corner_shrinked.y() < p.y < max_corner_shrinked.y()

                in_room_detections = [d for d in PERSON_DETECTIONS if _in_room(d['map_ps'].point)]

                rospy.loginfo("%d in room before clustering", len(in_room_detections))

                clusters = cluster_people(in_room_detections, np.array([6, 0]))

                return clusters

            # filter in room and perform clustering until we have 4 options
            try:
                person_detection_clusters = _get_clusters()
            except ValueError as e:
                rospy.logerr(e)
                robot.speech.speak("Mates, where are you?", block=False)
                return "failed"

            floorplan = cv2.imread(
                os.path.join(rospkg.RosPack().get_path('challenge_find_my_mates'), 'img/floorplan.png'))
            floorplan_height, floorplan_width, _ = floorplan.shape

            bridge = CvBridge()
            c_map = color_map(N=len(person_detection_clusters), normalized=True)
            for i, person_detection in enumerate(person_detection_clusters):
                image = bridge.imgmsg_to_cv2(person_detection['rgb'], "bgr8")
                roi = person_detection['person_detection'].face.roi
                roi_image = image[roi.y_offset:roi.y_offset + roi.height, roi.x_offset:roi.x_offset + roi.width]

                desired_height = 150
                height, width, channel = roi_image.shape
                ratio = float(height) / float(desired_height)
                calculated_width = int(float(width) / ratio)
                resized_roi_image = cv2.resize(roi_image, (calculated_width, desired_height))

                x = person_detection['map_ps'].point.x
                y = person_detection['map_ps'].point.y

                x_image_frame = 9.04 - x
                y_image_frame = 1.58 + y

                pixels_per_meter = 158

                px = int(pixels_per_meter * x_image_frame)
                py = int(pixels_per_meter * y_image_frame)

                cv2.circle(floorplan, (px, py), 3, (0,0,255), 5)

                try:
                    px_image = min(max(0, px - calculated_width / 2), floorplan_width - calculated_width - 1)
                    py_image = min(max(0, py - desired_height / 2), floorplan_height - desired_height - 1)

                    if px_image >= 0 and py_image >= 0:
                        #could not broadcast input array from shape (150,150,3) into shape (106,150,3)
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
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    LocatePeople(hero, 'living_room').execute()
