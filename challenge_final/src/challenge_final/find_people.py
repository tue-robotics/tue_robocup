# System
import math
import numpy as np
import time
from collections import deque

# ROS
import rospy
import smach
from geometry_msgs.msg import PointStamped

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills import get_robot_from_argv
from robot_skills.util import kdl_conversions

WAYPOINT_ID = "find_people_waypoint"
NUM_LOOKS = 2


class FindPeople(smach.StateMachine):
    def __init__(self, robot):
        """
        Finds the people in the living room and takes pictures of them. Put then in a data struct and put this in
        output keys.

        Output keys:
        * detected_people: same datastruct as was used in find my mates. Ask Rein for a pickled example

        :param robot: (Robot) api object
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

            @smach.cb_interface(outcomes=["done"], output_keys=["detected_people"])
            def detect_people(user_data):

                person_detections = []

                # with open('/home/rein/mates/floorplan-2019-07-05-11-06-52.pickle', 'r') as f:
                #     person_detections = pickle.load(f)
                #     rospy.loginfo("Loaded %d persons", len(person_detections))
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
                    "You are all looking great today! Keep looking at my camera.",
                    "I like it when everybody is staring at me!"
                ])

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

                user_data.detected_people = person_detections

                return 'done'

            smach.StateMachine.add('DETECT_PEOPLE',
                                   smach.CBState(detect_people),
                                   transitions={"done": "done"})

            # ToDo: add state: filter and cluster images


if __name__ == "__main__":

    rospy.init_node("test_furniture_inspection")

    # Robot
    _robot = get_robot_from_argv(index=1)

    # Test data
    user_data = smach.UserData()

    sm = FindPeople(robot=_robot)
    sm.execute(user_data)

    # Check output
    # noinspection PyProtectedMember
    rospy.loginfo("User data: {}".format(user_data._data))

    rospy.loginfo("Please check output above")
