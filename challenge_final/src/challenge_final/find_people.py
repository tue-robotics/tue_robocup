# System
import datetime
import math
import os
import pickle

import numpy as np
import time
from collections import deque

# ROS
import PyKDL as kdl
import rospy
import smach
from geometry_msgs.msg import PointStamped

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util import kdl_conversions

from .clustering import cluster_people

WAYPOINT_ID = "find_people_waypoint"
ROOM_ID = "living_room"
NUM_LOOKS = 2
MAX_PEOPLE = 6
SHRINK_X = 0.5
SHRINK_Y = 0.3


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


def _filter_and_cluster_images(robot, raw_person_detections, room_id):
    """
    Filters the raw detections so that only people within the room maintain. Next, it clusters the images
    so that only one image per person remains. This is stored in the user data

    :param raw_person_detections: raw detections
    :return: clusters
    """
    try:
        with open(os.path.expanduser(
            '~/floorplan-{}.pickle'.format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
        ), 'w') as f:
            pickle.dump(raw_person_detections, f)
    except:
        pass

    room_entity = robot.ed.get_entity(id=room_id)  # type: Entity
    room_volume = room_entity.volumes["in"]
    min_corner = room_entity.pose.frame * room_volume.min_corner
    max_corner = room_entity.pose.frame * room_volume.max_corner

    shrink_x = SHRINK_X
    shrink_y = SHRINK_Y
    min_corner_shrinked = kdl.Vector(min_corner.x() + shrink_x, min_corner.y() + shrink_y, 0)
    max_corner_shrinked = kdl.Vector(max_corner.x() - shrink_x, max_corner.y() - shrink_y, 0)

    rospy.loginfo('Start filtering from %d raw detections', len(raw_person_detections))

    def _get_clusters():
        def _in_room(p):
            return (min_corner_shrinked.x() < p.x < max_corner_shrinked.x() and
                    min_corner_shrinked.y() < p.y < max_corner_shrinked.y()
                    )

        in_room_detections = [d for d in raw_person_detections if _in_room(d['map_ps'].point)]

        rospy.loginfo("%d in room before clustering", len(in_room_detections))

        clusters = cluster_people(in_room_detections, np.array([6, 0]), n_clusters=MAX_PEOPLE)

        return clusters

    # filter in room and perform clustering until we have 4 options
    try:
        person_detection_clusters = _get_clusters()
    except ValueError as e:
        rospy.logerr(e)
        robot.speech.speak("Mates, where are you?", block=False)
        raise

    # Filter clusters to get the N best ones (not necessary if KMeans clustering algorithm is used)
    # def _score_cluster(cluster):
    #     """
    #     Scores the cluster based on an heuristic. It is desired that the cluster is as far in the room as is possible,
    #     assuming people are close to the far edge (x_max) and the right edge (y_min) of the room.
    #
    #     :param cluster: (dict)
    #     :return: (float) score
    #     """
    #     x_score = abs(max_corner_shrinked.x() - cluster["map_ps"].point.x)
    #     y_score = abs(- min_corner_shrinked.y() + cluster["map_ps"].point.y)
    #     return x_score if x_score < y_score else y_score
    # if len(person_detection_clusters) > MAX_PEOPLE:
    #     person_detection_clusters = sorted(person_detection_clusters, key=_score_cluster, reverse=True)[:MAX_PEOPLE]

    return person_detection_clusters


class FindPeople(smach.StateMachine):
    def __init__(self, robot, room_id=ROOM_ID):
        """
        Finds the people in the living room and takes pictures of them. Put then in a data struct and put this in
        output keys.

        Output keys:
        * detected_people: same datastruct as was used in find my mates. Ask Rein for a pickled example

        :param robot: (Robot) api object
        :param room_id: (str) identifies the room in which the people are detected
        """
        smach.StateMachine.__init__(self, outcomes=["done"], output_keys=["detected_people"])

        with self:

            # Move to start location
            smach.StateMachine.add("NAVIGATE_TO_START",
                                   states.NavigateToWaypoint(
                                       robot=robot,
                                       waypoint_designator=ds.EntityByIdDesignator(robot, WAYPOINT_ID), radius=0.3),
                                   transitions={"arrived": "DETECT_PEOPLE",
                                                "unreachable": "DETECT_PEOPLE",
                                                "goal_not_defined": "DETECT_PEOPLE"})

            @smach.cb_interface(outcomes=["done"], output_keys=["raw_detections"])
            def detect_people(user_data):

                person_detections = []

                #look_angles = np.linspace(-np.pi / 2, np.pi / 2, 8)  # From -pi/2 to +pi/2 to scan 180 degrees wide
                look_angles = np.linspace(-np.pi / 4, np.pi / 4, 4)  # From -pi/2 to +pi/2 to scan 180 degrees wide
                head_goals = [kdl_conversions.VectorStamped(x=100 * math.cos(angle),
                                                            y=100 * math.sin(angle),
                                                            z=1.5,
                                                            frame_id="/%s/base_link" % robot.robot_name)
                              for angle in look_angles]

                sentences = deque([
                    "Hi there mates, where are you, please look at me!",
                    "I am looking for my mates! Dippi dee doo! Pew pew!",
                    "You are all looking great today! Keep looking at my camera.",
                    "I like it when everybody is staring at me!"
                ])

                look_at_me_sentences = deque([
                    "Please look at me",
                    "Let me check you out",
                    "Your eyes are beautiful",
                    "Try to look pretty, your face will pop up on the screen later!"
                ])

                for _ in range(NUM_LOOKS):
                    sentences.rotate(1)
                    robot.speech.speak(sentences[0], block=False)
                    for head_goal in head_goals:
                        look_at_me_sentences.rotate(1)
                        robot.speech.speak(look_at_me_sentences[0], block=False)
                        robot.head.look_at_point(head_goal)
                        robot.head.wait_for_motion_done()
                        now = time.time()
                        rgb, depth, depth_info = robot.perception.get_rgb_depth_caminfo()

                        if rgb:

                            try:
                                persons = robot.perception.detect_person_3d(rgb, depth, depth_info)
                            except Exception as e:
                                rospy.logerr(e)
                                rospy.sleep(2.0)
                            else:
                                for person in persons:
                                    if person.face.roi.width > 0 and person.face.roi.height > 0:
                                        try:
                                            person_detections.append({
                                                "map_ps": robot.tf_listener.transformPoint("map", PointStamped(
                                                    header=rgb.header,
                                                    point=person.position
                                                )),
                                                "person_detection": person,
                                                "rgb": rgb
                                            })
                                        except Exception as e:
                                            rospy.logerr(
                                                "Failed to transform valid person detection to map frame: {}".format(e))

                            rospy.loginfo("Took %.2f, we have %d person detections now", time.time() - now,
                                          len(person_detections))

                rospy.loginfo("Detected %d persons", len(person_detections))

                user_data.raw_detections = person_detections

                return 'done'

            smach.StateMachine.add('DETECT_PEOPLE',
                                   smach.CBState(detect_people),
                                   transitions={"done": "FILTER_AND_CLUSTER"})

            # Filter and cluster images
            @smach.cb_interface(
                outcomes=["done", "failed"],
                input_keys=["raw_detections"],
                output_keys=["detected_people"])
            def filter_and_cluster_images(user_data):
                """
                Filters the raw detections so that only people within the room maintain. Next, it clusters the images
                so that only one image per person remains. This is stored in the user data

                :param user_data: (smach.UserData)
                :return: (str) Done
                """
                try:
                    user_data.detected_people = _filter_and_cluster_images(
                        robot, user_data.raw_detections, room_id)
                    return "done"
                except:
                    return "failed"

            smach.StateMachine.add('FILTER_AND_CLUSTER',
                                   smach.CBState(filter_and_cluster_images),
                                   transitions={"done": "done",
                                                "failed": "done"})  # ToDo: fallback
