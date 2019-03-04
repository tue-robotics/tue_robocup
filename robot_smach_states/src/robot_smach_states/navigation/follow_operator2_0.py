#!/usr/bin/env python

# System
import collections
import copy
import math
import random
import sys
import hmi

# ROS
import geometry_msgs  # Only used for publishing markers
import geometry_msgs.msg
import rospy
import smach
from visualization_msgs.msg import Marker

# TU/e
from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint
from robot_skills.util import kdl_conversions
from robot_smach_states.human_interaction import LearnOperator


class CrumbWaypoint:
    def __init__(self, crumb):
        self.crumb = crumb
        self.waypoint = None

    def __repr__(self):
        result = "{}".format(self.crumb)
        result += "\n{}".format(self.waypoint)

        return result


class Track(smach.State):
    def __init__(self, robot, _buffer):
        """ Constructor

        :param robot: robot object (amigo, sergio)
        :param _buffer:
        """
        smach.State.__init__(self,
                             outcomes=['track', 'no_track', 'aborted'],
                             input_keys=['operator_track_in'])
        self.counter = 0
        self._period = 0.3  # fix this magic number
        self._operator_pub = rospy.Publisher('/%s/follow_operator/operator_position' % robot.robot_name,
                                             geometry_msgs.msg.PointStamped, queue_size=10)
        self._robot = robot
        self._breadcrumb_distance = 0.1
        self._operator = None
        self._buffer = _buffer
        random.seed()

    def execute(self, userdata):
        """ A concurrent state in which the breadcrumbs, that are made in the other concurrent state, are followed.


        :return: track if the operator is followed flawlessly
                 no track if the operator is lost
        """
        if userdata.operator_track_in:
            operator = userdata.operator_track_in
            self._operator = operator
        else:
            operator = self._operator

        rospy.loginfo("Distance to goal: {}".format(self._robot.base.local_planner.getDistanceToGoal()))
        if random.randrange(20) == 1 and self._robot.base.local_planner.getDistanceToGoal() > 2.5:
            options = ["You seem to be in a hurry, is there ice cream in the groceries?",
                       "Not so fast!",
                       "Please slow down.",
                       "Why are you in such a hurry to leave me, don't you like me?"]
            sentence = random.choice(options)
            if sentence == options[3]:
                self._robot.speech.speak(sentence, mood='Sad')
            else:
                self._robot.speech.speak(sentence)

        if random.randrange(80) == 1:
            options = ["Your butt is looking wonderful!",
                       "Let's get your groceries!",
                       "I'm happy you are back, I was so lonely while you were shopping"]
            sentence = random.choice(options)
            if sentence == options[0]:
                self._robot.speech.speak(sentence, voice='gregory', mood='Excited')
            else:
                self._robot.speech.speak(sentence)

        rospy.loginfo("Trying to get operator with id: {}".format(operator.id))
        operator = self._robot.ed.get_entity(id=operator.id)
        if operator is None:
            rospy.loginfo("Could not find operator")
            _entities = self._robot.ed.get_entities()
            _laser_entity_ids = [e.id for e in _entities if "laser" in e.id]
            rospy.loginfo("Available laser IDs: {}".format(_laser_entity_ids))
            rospy.sleep(2)  # Crucial sleep! Do not remove!

            if self._operator:
                operator = self._robot.ed.get_entity(id=self._operator.id)

            if not operator:
                options = ["Don't move, I'm losing you.",
                           "Where did you go? Please stay where you are and I will find you.",
                           "Oh no I lost you for a second, please stay where you are and I will come and find you!"]
                sentence = random.choice(options)
                self._robot.speech.speak(sentence)
                self._robot.base.local_planner.cancelCurrentPlan()  # why is this here?
                return 'no_track'

            else:
                return 'track'

        else:
            rospy.loginfo("Found operator with id: {}".format(operator.id))
            self._buffer.append(operator)
            return 'track'


class FollowBread(smach.State):
    def __init__(self, robot, _buffer, operator_radius=1, lookat_radius=0.5):
        """

        :param robot: robot object (amigo, sergio)
        :param _buffer: the buffer deque that is made in the track state
        :param operator_radius: the assumed radius of the operator for position constraint
        :param lookat_radius: all breadcrumbs within the lookat_radius are deleted
        """
        smach.State.__init__(self,
                             outcomes=['follow_bread', 'no_follow_bread_ask_finalize', 'no_follow_bread_recovery'])
        self._robot = robot
        self._operator_radius = operator_radius
        self._lookat_radius = lookat_radius
        self._breadcrumb_distance = 0.1
        self._breadcrumb_pub = rospy.Publisher(
                '/%s/global_planner/visualization/markers/breadcrumbs' % robot.robot_name, Marker, queue_size=10)
        self._operator = None
        self._current_operator = collections.deque()
        self._buffer = _buffer
        self._breadcrumb = []
        self._resolution = 0.05
        self._timeout_count = 0
        # ros_plan = []

    def execute(self, userdata=None):
        """ A concurrent state machine which follows and afterwards deletes breadcrumbs from the buffer variable

        :return: follow_bread if the list of breadcrumbs is not empty
                 no_follow_bread_ask_finalize if the single remaining breadcrumb is the operator
                 no_follow_bread_recovery if buffer is completely empty
        """
        while self._buffer:

            if self.preempt_requested():
                return 'aborted'
            crumb = self._buffer.popleft()
            if not self._breadcrumb or self._breadcrumb[-1].crumb.distance_to_2d(
                crumb._pose.p) > self._breadcrumb_distance:
                self._breadcrumb.append(CrumbWaypoint(crumb))
        rospy.loginfo("Self._breadcrumb length: {}".format(len(self._breadcrumb)))

        # We don't want to base this on distance since that relies on the operator behavior
        # if self._robot.base.local_planner.getDistanceToGoal() > 0.65:  # 2.0:  # len(buffer) > 5:
        #     self._have_followed = True

        robot_position = self._robot.base.get_location().frame

        if self._breadcrumb:
            self._operator = self._breadcrumb[-1].crumb

        else:
            return 'no_follow_bread_recovery'
        #
        # if self._operator:
        #     operator = self._robot.ed.get_entity(id=self._operator.id)

        # while True:  # Should be timer, I think
        try:
            _, semantics = self._robot.hmi.query(description="", grammar="T[true] -> %s stop" % self._robot.robot_name, target="T")
            if semantics:
                return 'no_follow_bread_ask_finalize'
        except hmi.TimeoutException:
            rospy.logwarn("[HMC] Listening timeout")

        # return 'no_follow_bread_ask_finalize'

        # if operator:
        #     rospy.loginfo("Distance to operator: {}".format(operator.distance_to_2d(robot_position.p)))
        #     if self._have_followed and operator.distance_to_2d(robot_position.p) < 1.0:
        #         self._have_followed = False
        #         return 'no_follow_bread_ask_finalize'

        # self._breadcrumb = [crwp for crwp in self._breadcrumb
        #                     if crwp.crumb.distance_to_2d(robot_position.p) > self._lookat_radius]

        current_index = -1
        for i, crumb in enumerate(self._breadcrumb):
            if crumb.crumb.distance_to_2d(robot_position.p) < self._lookat_radius:
                current_index = i

        # throw away all breadcrumbs before where we are
        self._breadcrumb = self._breadcrumb[current_index+1:]

        f = self._robot.base.get_location().frame
        previous_point = f.p
        operator_position = self._operator._pose.p

        ''' Define end goal constraint, solely based on the (old) operator position '''
        p = PositionConstraint()
        p.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2" % (operator_position.x(), operator_position.y(),
                                                       self._operator_radius)
        o = OrientationConstraint()
        o.frame = self._operator.id
        ''' Calculate global plan from robot position, through breadcrumbs, to the operator '''

        for crumb_waypoint in self._breadcrumb:
            crumb = crumb_waypoint.crumb
            diff = crumb._pose.p - previous_point
            dx, dy = diff.x(), diff.y()
            length = crumb.distance_to_2d(previous_point)

            if length != 0:
                dx_norm = dx / length
                dy_norm = dy / length
                yaw = math.atan2(dy, dx)
                start = 0
                end = int(length / self._resolution)

                for i in range(start, end):
                    x = previous_point.x() + i * dx_norm * self._resolution
                    y = previous_point.y() + i * dy_norm * self._resolution
                    kdl_pose = kdl_conversions.kdl_frame_stamped_from_XYZRPY(x=x, y=y, z=0, yaw=yaw)
                    crumb_waypoint.waypoint = kdl_conversions.kdl_frame_stamped_to_pose_stamped_msg(kdl_pose)
            previous_point = copy.deepcopy(crumb._pose.p)

        if len(self._breadcrumb) > 0:
            ros_plan = [crwp.waypoint for crwp in self._breadcrumb]
            if not self._robot.base.global_planner.checkPlan(ros_plan):
                rospy.loginfo("Breadcrumb plan is blocked, removing blocked points")
                rospy.loginfo("Breadcrumbs before removing blocked points: {}".format(len(ros_plan)))
                ros_plan = [crwp.waypoint for crwp in self._breadcrumb
                            if self._robot.base.global_planner.checkPlan([crwp.waypoint])]
                rospy.loginfo("Breadcrumbs length after removing blocked points: {}".format(len(ros_plan)))

            # if the new ros plan is empty, we do have an operator but there are no breadcrumbs that we can reach
            # if not ros_plan:
            #     return 'no_follow_bread_recovery'
        else:
            return 'no_follow_bread_recovery'

        buffer_msg = Marker()
        buffer_msg.type = Marker.POINTS
        buffer_msg.header.stamp = rospy.get_rostime()
        buffer_msg.header.frame_id = "/map"
        buffer_msg.id = 0
        buffer_msg.action = Marker.DELETEALL
        self._breadcrumb_pub.publish(buffer_msg)

        buffer_msg = Marker()
        buffer_msg.type = Marker.POINTS
        buffer_msg.scale.x = 0.05
        buffer_msg.scale.y = 0.05
        buffer_msg.header.stamp = rospy.get_rostime()
        buffer_msg.header.frame_id = "/map"
        buffer_msg.color.a = 1
        buffer_msg.color.r = 1
        buffer_msg.color.g = 0
        buffer_msg.color.b = 0
        # buffer_msg.lifetime = rospy.Time(1)
        buffer_msg.id = 0
        buffer_msg.action = Marker.ADD

        for crumb in self._breadcrumb:
            buffer_msg.points.append(crumb.waypoint.pose.position)

        self._breadcrumb_pub.publish(buffer_msg)

        # line_strip = Marker()
        # line_strip.type = Marker.LINE_STRIP
        # line_strip.scale.x = 0.05
        # line_strip.header.frame_id = "/map"
        # line_strip.header.stamp = rospy.Time.now()
        # line_strip.color.a = 1
        # line_strip.color.r = 0
        # line_strip.color.g = 1
        # line_strip.color.b = 1
        # line_strip.id = 0
        # line_strip.action = Marker.ADD
        #
        # for crwp in ros_plan:
        #     line_strip.points.append(crwp)

        #self._plan_marker_pub.publish(line_strip)
        self._robot.base.local_planner.setPlan(ros_plan, p, o)
        rospy.sleep(rospy.Duration(0.5))
        return 'follow_bread'


class Recovery(smach.State):
    def __init__(self, robot, lost_timeout=60, lost_distance=0.8):
        """

        :param robot: robot object (amigo, sergio)
        :param lost_timeout: time allowed to take to find the operator
        :param lost_distance: radius in which to find the laser entity of the operator based on the converted location
                              from the RGBD image
        """
        smach.State.__init__(self, outcomes=['Failed', 'follow'],
                             output_keys=['recovered_operator'])
        self._robot = robot
        self._operator_name = "operator"
        self._lost_timeout = lost_timeout
        self._lost_distance = lost_distance
        self._face_pos_pub = rospy.Publisher('/%s/follow_operator/operator_detected_face' % robot.robot_name,
                                             geometry_msgs.msg.PointStamped, queue_size=10)

    def execute(self, userdata):
        """
        :return: Failed: when taking too long, can't transform RGBD image to point or when preempt requested
                 follow: When successfully recovered operator
        """
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("%s, please look at me while I am looking for you" % self._operator_name, block=False)

        start_time = rospy.Time.now()

        look_distance = 2.0
        look_angles = [0.0,
                       math.pi / 6,
                       math.pi / 4,
                       math.pi / 2.3,
                       0.0,
                       -math.pi / 6,
                       -math.pi / 4,
                       -math.pi / 2.3]
        head_goals = [kdl_conversions.VectorStamped(x=look_distance * math.cos(angle),
                                                    y=look_distance * math.sin(angle), z=1.7,
                                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in look_angles]

        i = 0
        while (rospy.Time.now() - start_time).to_sec() < self._lost_timeout:
            if self.preempt_requested():
                return 'Failed'

            self._robot.head.look_at_point(head_goals[i])
            i += 1
            if i == len(head_goals):
                i = 0
            self._robot.head.wait_for_motion_done()
            raw_detections = self._robot.perception.detect_faces()
            best_detection = self._robot.perception.get_best_face_recognition(raw_detections, "operator", probability_threshold=3.0)

            rospy.loginfo("best_detection = {}".format(best_detection))
            if best_detection:
                roi = best_detection.roi
                try:
                    operator_pos_kdl = self._robot.perception.project_roi(roi=roi, frame_id="map")
                except Exception as e:
                    rospy.logerr("head.project_roi failed: %s", e)
                    return 'Failed'
                operator_pos_ros = kdl_conversions.kdl_vector_stamped_to_point_stamped(operator_pos_kdl)
                self._face_pos_pub.publish(operator_pos_ros)

                recovered_operator = self._robot.ed.get_closest_laser_entity(radius=self._lost_distance,
                                                                             center_point=operator_pos_kdl,
                                                                             ignore_z=True)
                if recovered_operator:
                    print
                    "Found one!"
                    self._operator_id = recovered_operator.id
                    print
                    "Recovered operator id: %s" % self._operator_id
                    self._operator = recovered_operator
                    self._robot.speech.speak("There you are! Go ahead, I'll follow you again", block=False)
                    self._robot.head.close()
                    self._time_started = rospy.Time.now()
                    userdata.recovered_operator = recovered_operator
                    return 'follow'
                else:
                    print
                    "Could not find an entity {} meter near {}".format(self._lost_distance, operator_pos_kdl)

        self._robot.head.close()
        return 'Failed'


class FollowOperator2(smach.StateMachine):
    def __init__(self, robot):
        """
        :param robot: robot object (amigo, sergio)
        """
        smach.StateMachine.__init__(self, outcomes=['Done', 'Failed', 'Aborted'])
        self.robot = robot
        self.userdata.operator = None

        with self:
            smach.StateMachine.add('LEARN_OPERATOR', LearnOperator(robot),
                                   transitions={'done': 'CON_FOLLOW',
                                                'failed': 'Failed',
                                                'aborted': 'Aborted'
                                                },
                                   remapping={'operator_learn_in': 'operator', 'operator_learn_out': 'operator'})

            smach.StateMachine.add('RECOVERY', Recovery(robot),
                                   transitions={'Failed': 'Failed',
                                                'follow': 'CON_FOLLOW'},
                                   remapping={'recovered_operator': 'operator'})

            sm_con = smach.Concurrence(outcomes=['recover_operator', 'ask_finalize', 'keep_following', 'Aborted'],
                                       default_outcome='keep_following',
                                       outcome_map={'ask_finalize': {'FOLLOWBREAD': 'no_follow_bread_ask_finalize',
                                                                     'TRACK': 'track'},
                                                    'recover_operator': {'FOLLOWBREAD': 'no_follow_bread_recovery',
                                                                         'TRACK': 'no_track'},
                                                    'Aborted': {'TRACK': 'aborted'}},
                                       input_keys=['operator'])

            _buffer = collections.deque()
            sm_con.userdata.operator = None

            with sm_con:
                smach.Concurrence.add('TRACK', Track(robot, _buffer), remapping={'operator_track_in': 'operator'})

                smach.Concurrence.add('FOLLOWBREAD', FollowBread(robot, _buffer))

            smach.StateMachine.add('CON_FOLLOW', sm_con,
                                   transitions={'recover_operator': 'RECOVERY',
                                                'ask_finalize': 'Done',
                                                'keep_following': 'CON_FOLLOW',
                                                'Aborted': 'Aborted'})


if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print
        "Please provide robot name as argument."
        exit(1)

    if robot_name == "amigo":
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == "sergio":
        from robot_skills.sergio import Sergio as Robot

    rospy.init_node('test_follow_operator')
    robot = Robot()
    sm = FollowOperator2(robot)
    sm.execute()
