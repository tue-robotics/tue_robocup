#!/usr/bin/python

import math
from operator import itemgetter

import numpy as np
import robot_smach_states as states
import rospy
import smach
# from challenge_restaurant.srv import GetNormalScore
from robot_skills.util.kdl_conversions import FrameStamped, VectorStamped, kdl_frame_to_pose_msg
from robot_skills.util.shape import RightPrism
from visualization_msgs.msg import Marker


def _get_area(convex_hull):
    pts = [[c.x(), c.y()] for c in convex_hull]
    lines = np.hstack([pts, np.roll(pts, -1, axis=0)])
    area = 0.5 * abs(sum(x1 * y2 - x2 * y1 for x1, y1, x2, y2 in lines))
    return area


# class AutomaticSideDetection2(smach.State):
#     """ State to automatically detect whether a table or similar is left or right of the robot """
#
#     def __init__(self, robot):
#         """ Constructor
#
#         :param robot: robot object
#         """
#         look_x = 0.2
#         look_y = 1.5
#         self._sides = {
#             'left': VectorStamped(look_x, look_y, z=0, frame_id="/" + robot.robot_name + "/base_link"),
#             'right': VectorStamped(look_x, -look_y, z=0, frame_id="/" + robot.robot_name + "/base_link"),
#         }
#         smach.State.__init__(self, outcomes=self._sides.keys())
#
#         self._robot = robot
#         self._get_normal_score = rospy.ServiceProxy('/' + self._robot.robot_name + '/top_kinect/get_normal_score',
#                                                     GetNormalScore)
#
#     def execute(self, userdata=None):
#         scores = {}
#         for side, vs in self._sides.items():
#             # Look at the side
#             rospy.loginfo("looking at side %s" % side)
#             self._robot.head.look_at_point(vs)
#             self._robot.head.wait_for_motion_done()
#             rospy.sleep(0.2)
#
#             base_loc = self._robot.base.get_location()
#             base_position = base_loc.frame.p
#
#             score = self._get_normal_score().score
#             rospy.loginfo('Normal score: %f', score)
#             scores[side] = score
#
#         min_side, min_score = max(scores.items(), key=itemgetter(1))
#         return min_side
#
#
# class AutomaticSideDetection(smach.State):
#     """ State to automatically detect whether a table or similar is left or right of the robot """
#     def __init__(self, robot, background_padding=0.3, look_x=0.2, look_y=1.5, max_radius=1.5, min_area=0.3):
#         """ Constructor
#
#         :param robot: robot object
#         :param background_padding:
#         :param look_x:
#         :param look_y:
#         :param max_radius:
#         :param min_area:
#         """
#         self._sides = {
#             "left": {
#                 "x": look_x,
#                 "y": look_y,
#                 "score": {},
#                 "entities": []
#             },
#             "right": {
#                 "x": look_x,
#                 "y": -look_y,
#                 "score": {},
#                 "entities": []
#             },
#         }
#         smach.State.__init__(self, outcomes=self._sides.keys())
#         self._robot = robot
#         self._background_padding = background_padding
#         self._max_radius = max_radius
#         self._min_area = min_area
#
#     def _get_head_goal(self, spec):
#         vs = VectorStamped(spec["x"], spec["y"], z=0, frame_id="/"+self._robot.robot_name+"/base_link")
#         return vs
#
#     def _inspect_sides(self):
#         for side, spec in self._sides.iteritems():
#             # Look at the side
#             rospy.loginfo("looking at side %s" % side)
#             vs = self._get_head_goal(spec)
#             self._robot.head.look_at_point(vs)
#             self._robot.head.wait_for_motion_done()
#             rospy.sleep(0.2)
#
#             base_loc = self._robot.base.get_location()
#             base_position = base_loc.frame.p
#
#             # Update kinect
#             try:
#                 rospy.loginfo("Updating kinect for side %s" % side)
#                 # kinect_update = self._robot.ed.update_kinect(background_padding=self._background_padding)
#                 kinect_update = self._robot.ed.update_kinect(background_padding=self._background_padding)
#             except:
#                 rospy.logerr("Could not update_kinect")
#                 continue
#
#             ents = [self._robot.ed.get_entity(id=id_) for id_ in set(kinect_update.new_ids + kinect_update.updated_ids)]
#             ents = [e for e in ents if e is not None]
#
#             rospy.loginfo("Found %d entities for side %s" % (len(ents), side))
#
#             # Filter subset
#             self._sides[side]["entities"] = [e for e in ents if self._subset_selection(base_position, e)]
#
#             # Score entities
#             self._sides[side]["score"]["area_sum"] = sum([self._score_area(e) for e in self._sides[side]["entities"]])
#             self._sides[side]["score"]["min_distance"] = self._score_closest_point(base_position,
#                                                                                    self._sides[side]["entities"])
#
#             vs.vector.z(0.8)
#             self._robot.head.look_at_point(vs)
#             self._robot.head.wait_for_motion_done()
#             rospy.sleep(0.2)
#
#     def _subset_selection(self, base_position, e):
#         distance = e.distance_to_2d(base_position)
#         return distance < self._max_radius
#
#     @staticmethod
#     def _score_area(e):
#         """ Scores the area of an entity. If the shape is not a convex hull, it's not what were looking for and 0
#         is returned
#         """
#         if isinstance(e.shape, RightPrism):
#             return _get_area(e.shape.convex_hull)
#         else:
#             return 0.0
#
#     def _score_closest_point(self, base_position, entities):
#         distances = [e.distance_to_2d(base_position) for e in entities]
#         if distances:
#             min_distance = min(distances)
#         else:
#             min_distance = self._max_radius
#
#         return (self._max_radius - min_distance) / self._max_radius
#
#     def _get_best_side(self):
#
#         best_side = None
#         for side, spec in self._sides.iteritems():
#             end_score = self._sides[side]["score"]["area_sum"] + \
#                         self._sides[side]["score"]["min_distance"]
#             rospy.loginfo("Side %s scoring (%f): %s" % (side, end_score, self._sides[side]["score"]))
#
#             if best_side is None or self._sides[side]["score"] > self._sides[best_side]["score"]:
#                 best_side = side
#
#         return best_side
#
#     def execute(self, userdata=None):
#         rospy.sleep(0.2)
#         self._inspect_sides()
#
#         best_side = self._get_best_side()
#
#         return best_side


class StoreWaypoint(smach.State):
    """ Stores a waypoint """
    def __init__(self, robot, location_id):
        """ Constructor

        :param robot: robot object
        :param location_id: string identifying the location to store
        """
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        self._location_id = location_id
        self._waypoint_pub = rospy.Publisher("/restaurant_waypoints", Marker, queue_size=10)

    def execute(self, userdata=None):

        # Store current base position
        base_loc = self._robot.base.get_location()
        base_pose = base_loc.frame

        # Create automatic side detection state and execute
        # self._robot.speech.speak("I am now going to look for the table", block=False)
        # automatic_side_detection = AutomaticSideDetection2(self._robot)
        # side = automatic_side_detection.execute({})
        # self._robot.head.look_at_standing_person()

        # self._robot.speech.speak("The {} is to my {}".format(self._location_id, side))

        # self._robot.head.cancel_goal()

        # if side == "left":
        #     base_pose.M.DoRotZ(math.pi / 2)
        # elif side == "right":
        #     base_pose.M.DoRotZ(-math.pi / 2)

        # Add to param server
        loc_dict = {'x': base_pose.p.x(), 'y': base_pose.p.y(), 'phi': base_pose.M.GetRPY()[2]}
        rospy.set_param("/restaurant_locations/{name}".format(name=self._location_id), loc_dict)

        self._visualize_location(base_pose, self._location_id)
        self._robot.ed.update_entity(id=self._location_id, frame_stamped=FrameStamped(base_pose, "/map"),
                                     type="waypoint")

        return "done"

    def _visualize_location(self, base_pose, location):
        """
        Visualize a marker on the base_pose with the text 'location' and rotated to the correct side.

        :param base_pose: kdl Frame (in map) where the waypoint is located
        :param location: The name of the location as a label
        :return:
        """
        # Convert KDL object to geometry message
        base_pose = kdl_frame_to_pose_msg(base_pose)

        # Create the marker
        m = Marker()
        if location == "one":
            m.id = 1
            m.color.r = 1
        elif location == "two":
            m.id = 2
            m.color.g = 1
        elif location == "three":
            m.id = 3
            m.color.b = 1
        else:
            m.id = 4
            m.color.r = 1
            m.color.g = 1
            m.color.b = 1
        m.color.a = 1
        m.pose = base_pose
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.type = 0  # Arrow
        m.scale.x = 1.0
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.action = 0
        m.ns = "arrow"
        self._waypoint_pub.publish(m)
        m.type = 9
        m.text = location
        m.ns = "text"
        m.pose.position.z = 0.5
        self._waypoint_pub.publish(m)


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted'])
    robot.ed.reset()

    with sm:
        smach.StateMachine.add('STORE_WAYPOINT', StoreWaypoint(robot), transitions={'done': 'RESET',
                                                                                    'continue': 'RESET'})
        smach.StateMachine.add('RESET', states.ResetED(robot), transitions={'done': 'WAIT_SAY'})

    return sm

if __name__ == '__main__':
    rospy.init_node('automatic_side_detection')

    from robot_skills.amigo import Amigo
    robot = Amigo()
    robot.ed.reset()

    sm = smach.StateMachine(outcomes=['done', 'aborted'])
    with sm:
        smach.StateMachine.add('STORE_WAYPOINT', StoreWaypoint(robot, "kitchen"), transitions={'done': 'RESET',
                                                                                               'continue': 'RESET'})
        smach.StateMachine.add('RESET', states.ResetED(robot), transitions={'done': 'done'})

    # states.startup(setup_statemachine, challenge_name="automatic_side_detection")
    sm.execute()

