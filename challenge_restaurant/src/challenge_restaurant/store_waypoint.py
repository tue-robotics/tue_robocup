#!/usr/bin/python

import numpy as np

from geometry_msgs.msg import PoseStamped
from pykdl_ros import FrameStamped
import robot_smach_states as states
import rospy
import smach

import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros

from visualization_msgs.msg import Marker


def _get_area(convex_hull):
    pts = [[c.x(), c.y()] for c in convex_hull]
    lines = np.hstack([pts, np.roll(pts, -1, axis=0)])
    area = 0.5 * abs(sum(x1 * y2 - x2 * y1 for x1, y1, x2, y2 in lines))
    return area


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
        self._robot.ed.update_entity(uuid=self._location_id, frame_stamped=FrameStamped(base_pose, rospy.Time.now(),
                                                                                        "map"),
                                     etype="waypoint")

        return "done"

    def _visualize_location(self, base_pose, location):
        """
        Visualize a marker on the base_pose with the text 'location' and rotated to the correct side.

        :param base_pose: kdl Frame (in map) where the waypoint is located
        :param location: The name of the location as a label
        :return:
        """
        # Convert KDL object to geometry message
        base_pose = FrameStamped(base_pose, rospy.Time.now(), "map")
        base_pose = tf2_ros.convert(base_pose, PoseStamped).pose

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
        smach.StateMachine.add('RESET', states.reset.ResetED(robot), transitions={'done': 'WAIT_SAY'})

    return sm


if __name__ == '__main__':
    from robot_skills import get_robot
    rospy.init_node('automatic_side_detection')

    robot = get_robot('hero', '1')
    robot.ed.reset()

    sm = smach.StateMachine(outcomes=['done', 'aborted'])
    with sm:
        smach.StateMachine.add('STORE_WAYPOINT', StoreWaypoint(robot, "kitchen"), transitions={'done': 'RESET',
                                                                                               'continue': 'RESET'})
        smach.StateMachine.add('RESET', states.reset.ResetED(robot), transitions={'done': 'done'})

    # states.startup(setup_statemachine, challenge_name="automatic_side_detection")
    sm.execute()
