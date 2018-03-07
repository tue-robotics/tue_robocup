#!/usr/bin/env python

import smach
import rospy
import sys
import smach_ros
import math
import collections
from robot_smach_states.util.startup import startup
from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint
from robot_skills.util import kdl_conversions
import PyKDL as kdl
import geometry_msgs  # Only used for publishing markers
import geometry_msgs.msg
def vector_stampeds_to_point_stampeds(vector_stampeds):
    return map(kdl_conversions.kdlVectorStampedToPointStamped, vector_stampeds)
def frame_stampeds_to_pose_stampeds(frame_stampeds):
    return map(kdl_conversions.kdlFrameStampedToPoseStampedMsg, frame_stampeds)
import copy
from visualization_msgs.msg import Marker
from hmi import TimeoutException

class LearnOperator(smach.State):
    def __init__(self, robot, operator_timeout=20, ask_follow=True, learn_face=True, learn_person_timeout = 10.0):
        smach.State.__init__(self, outcomes=['follow', 'Failed'],
                             input_keys=['operator_learn_in'],
                             output_keys=['operator_learn_out'])
        self._robot = robot
        self._operator_timeout = operator_timeout
        self._ask_follow = ask_follow
        self._learn_face = learn_face
        self._learn_person_timeout = learn_person_timeout
        self._operator_name = "operator"

    def execute(self, userdata):
        start_time = rospy.Time.now()
        self._robot.head.look_at_standing_person()
        operator = userdata.operator_learn_in
        # import pdb; pdb.set_trace()
        while not operator:
            if self.preempt_requested():
                return 'Failed'

            if(rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return 'Failed'

            operator = self._robot.ed.get_closest_laser_entity(
                radius=0.5,
                center_point=kdl_conversions.VectorStamped(x=1.0, y=0, z=1,
                                                           frame_id="/%s/base_link" % self._robot.robot_name))
            rospy.loginfo("Operator: {op}".format(op=operator))
            if not operator:
                self._robot.speech.speak("Please stand in front of me")
            else:
                if self._learn_face:
                    self._robot.speech.speak("Please look at me while I learn to recognize you.",
                                             block=True)
                    self._robot.head.look_at_standing_person()
                    learn_person_start_time = rospy.Time.now()
                    num_detections = 0
                    while num_detections < 5: # 5:
                        if self._robot.perception.learn_person(self._operator_name):
                            #self._robot.speech.speak("Succesfully detected you %i times" % (num_detections + 1),block = False)
                            print("Succesfully detected you %i times" % (num_detections + 1))
                            num_detections += 1
                        elif (rospy.Time.now() - learn_person_start_time).to_sec() > self._learn_person_timeout:
                            self._robot.speech.speak("Please stand in front of me and look at me")
                            operator = None
                            break
        print "We have a new operator: %s" % operator.id
        self._robot.speech.speak("Gotcha! I will follow you!", block=False)
        self._robot.head.close()
        userdata.operator_learn_out = operator
        return 'follow'

class Track(smach.State):  # Updates the breadcrumb path
    def __init__(self, robot):
        smach.State.__init__(self,
                              outcomes=['track', 'no_track'],
                              input_keys=['buffer_track_in', 'operator_track_in'],
                              output_keys=['buffer_track_out'])
        self.counter = 0
        self._period = 2          #fix this magic number
        self._operator_pub = rospy.Publisher('/%s/follow_operator/operator_position' % robot.robot_name,
                                             geometry_msgs.msg.PointStamped, queue_size=10)
        self._robot = robot
        self._operator_distance = None
        self._breadcrumb_distance = 0.5
        self._last_operator = None
        self._last_operator_id = None

    def execute(self, userdata):
        operator = userdata.operator_track_in
        buffer = userdata.buffer_track_in

        if operator.id:
            operator = self._robot.ed.get_entity(id=operator.id )
        else:
            operator = None

        if operator:
            if (rospy.Time.now().to_sec() - operator.last_update_time) > self._period:
                self._robot.speech.speak("Not so fast!")

            self._last_operator = operator
            self._last_operator_id = operator.id

            operator_pos = geometry_msgs.msg.PointStamped()
            operator_pos.header.stamp = rospy.get_rostime()
            operator_pos.header.frame_id = operator.id
            operator_pos.point.x = 0.0
            operator_pos.point.y = 0.0
            operator_pos.point.z = 0.0

            f = self._robot.base.get_location().frame #changes made in tf_server/src/tf_server/tf_client.py & robot_skills/src/robot_skills/robot.py
            self._operator_distance = self._last_operator.distance_to_2d(f.p)

            if buffer:
                if buffer[-1].distance_to_2d(operator._pose.p) < self._breadcrumb_distance:
                    buffer[-1] = operator
                else:
                    buffer.append(operator)
            else:
                buffer.append(operator) # collections.deque([operator_pos])
            # buffer.append([operator_pos])
            userdata.buffer_track_out = buffer
            return 'track'

        else:
        #    if not self._last_operator:                                We leave this out for now, maybe it is necessary to implement it later.
        #         if self._backup_register():
        #             # If the operator is still tracked, it is also the last_operator
        #             self._last_operator = self._operator
        #
        #             operator_pos = geometry_msgs.msg.PointStamped()
        #             operator_pos.header.stamp = rospy.get_rostime()
        #             operator_pos.header.frame_id = self._operator_id
        #             operator_pos.point.x = 0.0
        #             operator_pos.point.y = 0.0
        #             operator_pos.point.z = 0.0
        #             self._operator_pub.publish(operator_pos)
        #
        #             f = self._robot.base.get_location().frame
        #             self._operator_distance = self._last_operator.distance_to_2d(f.p)
        #
        #             return True
        #         else:
        #             self._robot.speech.speak("I'm sorry, but I couldn't find a person to track")
        #
        #     f = self._robot.base.get_location().frame
        #     self._operator_distance = self._last_operator.distance_to_2d(f.p)
        #     # If the operator is lost, check if we still have an ID
        #     if self._operator_id:
        #         # At the moment when the operator is lost, tell him to slow down and clear operator ID
        #         self._operator_id = None
        #         self._robot.speech.speak("Stop! I lost you! Until I find you again, please wait there.", block=False)
        #     return False


        #     while not operator:
        #         operator = self._robot.ed.get_entity(id=self._last_operator_id)
        #         rospy.sleep(2)
        #         self._robot.speech.speak(
        #         "Oh no I lost you for a second, please stay where you are and I will come and find you again!")
        #         return 'no_track'
        #     buffer.append(operator)
        #     return 'track'

            # self._robot.speech.speak(
            print("Oh no I lost you for a second, please stay where you are and I will come and find you again!")

            return 'no_track'


class FollowBread(smach.State):
    def __init__(self, robot, operator_radius=1, lookat_radius=1.2):
        smach.State.__init__(self,
                             outcomes=['follow_bread', 'no_follow_bread'],
                             input_keys=['buffer_follow_in'],
                             output_keys=['buffer_follow_out'])
        self._robot = robot
        self._operator_radius = operator_radius
        self._lookat_radius = lookat_radius
        self._breadcrumb_pub = rospy.Publisher('/%s/follow_operator/breadcrumbs' % robot.robot_name, Marker,
                                               queue_size=10)
        self._plan_marker_pub = rospy.Publisher(
            '/%s/global_planner/visualization/markers/global_plan' % robot.robot_name, Marker, queue_size=10)
        self._have_followed = False

    def execute(self, userdata):
        buffer = userdata.buffer_follow_in
        # print list(userdata.buffer)
        if len(buffer) > 5: #5
            self._have_followed = True

        while not buffer:
            if not self._have_followed:
                rospy.sleep(5)      #magic number
            else:
                break

        newest_crumb = buffer[0]
        operator = buffer[-1]
        if len(buffer) == 1:
            last_operator = buffer[-1]
        else:
            last_operator = buffer[-2]
        robot_position = self._robot.base.get_location().frame
        while len(buffer) > 0 and newest_crumb.distance_to_2d(robot_position.p) < self._lookat_radius + 0.1:
            buffer.popleft()

        print "Buffer length after popping crumbs that are to close %i" % len(buffer)
        current_operator = self._robot.ed.get_entity(id=operator.id)
        if not buffer and self._have_followed and current_operator.distance_to_2d(robot_position.p) < 0.5:
            # rospy.sleep(1)
            print current_operator.distance_to_2d(robot_position.p)
            self._have_followed = False
            return 'no_follow_bread'
        self._robot.head.cancel_goal()
        f = self._robot.base.get_location().frame
        robot_position = f.p
        operator_position = last_operator._pose.p

        ''' Define end goal constraint, solely based on the (old) operator position '''
        p = PositionConstraint()
        p.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2" % (operator_position.x(), operator_position.y(),
                                                       self._operator_radius)

        o = OrientationConstraint()
        if operator.id:
            o.frame = operator.id
        else:
            o.frame = 'map'
            o.look_at = kdl_conversions.kdlVectorToPointMsg(last_operator.pose.frame.p)

        ''' Calculate global plan from robot position, through breadcrumbs, to the operator '''
        res = 0.05      ##magic number number 2
        kdl_plan = []
        previous_point = robot_position

        if operator:
            buffer.append(operator)
        else:
            buffer.append(last_operator)
        for crumb in buffer:
            # assert isinstance(crumb, Entity)
            diff = crumb._pose.p - previous_point
            dx, dy = diff.x(), diff.y()
            length = crumb.distance_to_2d(previous_point)

            if length != 0:
                dx_norm = dx / length
                dy_norm = dy / length
                yaw = math.atan2(dy, dx)

                start = 0
                end = int(length / res)

                for i in range(start, end):
                    x = previous_point.x() + i * dx_norm * res
                    y = previous_point.y() + i * dy_norm * res
                    kdl_plan.append(kdl_conversions.kdlFrameStampedFromXYZRPY(x=x, y=y, z=0, yaw=yaw))

            previous_point = copy.deepcopy(crumb._pose.p)

        # Delete the elements from the plan within the operator radius from the robot
        cutoff = int(self._operator_radius / (2.0 * res))
        if len(kdl_plan) > cutoff:
            del kdl_plan[-cutoff:]

        ros_plan = frame_stampeds_to_pose_stampeds(kdl_plan)
        # Check if plan is valid. If not, remove invalid points from the path
        if not self._robot.base.global_planner.checkPlan(ros_plan):
            print "Breadcrumb plan is blocked, removing blocked points"
            # Go through plan from operator to robot and pick the first unoccupied point as goal point
            ros_plan = [point for point in ros_plan if self._robot.base.global_planner.checkPlan([point])]
            # print ros_plan
            print "Length of rosplan %i" % len(ros_plan)

        buffer_msg = Marker()
        buffer_msg.type = Marker.POINTS
        buffer_msg.scale.x = 0.05
        buffer_msg.header.stamp = rospy.get_rostime()
        buffer_msg.header.frame_id = "/map"
        buffer_msg.color.a = 1
        buffer_msg.color.r = 0
        buffer_msg.color.g = 1
        buffer_msg.color.b = 1
        buffer_msg.lifetime = rospy.Time(1.0)
        buffer_msg.id = 0
        buffer_msg.action = Marker.ADD

        for crumb in buffer:
            buffer_msg.points.append(kdl_conversions.kdlVectorToPointMsg(crumb.pose.frame.p))

        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.scale.x = 0.05
        line_strip.header.frame_id = "/map"
        line_strip.header.stamp = rospy.Time.now()
        line_strip.color.a = 1
        line_strip.color.r = 0
        line_strip.color.g = 1
        line_strip.color.b = 1
        line_strip.id = 0
        line_strip.action = Marker.ADD

        # Push back all pnts
        for pose_stamped in ros_plan:
            line_strip.points.append(pose_stamped.pose.position)

        self._plan_marker_pub.publish(line_strip)
        self._breadcrumb_pub.publish(buffer_msg)
        # self._visualize_plan(ros_plan)
        self._robot.base.local_planner.setPlan(ros_plan, p, o)
        userdata.buffer_follow_out = buffer
        print "Buffer length at the end of the follow state %i" % len(buffer)
        return 'follow_bread'


class AskFinalize(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,  outcomes=['follow', 'Done'])
        self._robot = robot

    def execute(self, userdata=None):
        sentence = "Are we there yet?"
        self._robot.speech.speak(sentence, block=True)
        try:
            answer = self._robot.hmi.query(sentence, "T -> yes | no", "T")
        except TimeoutException as e:
            self._robot.speech.speak("I did not hear you!")
            rospy.sleep(2)
        else:
            if answer.sentence == "yes":
                self._robot.speech.speak("We reached our final destination!")
                return 'Done'
            else:
                return 'follow'


class Recovery(smach.State):
    def __init__(self, robot, lost_timeout=60, lost_distance=0.8):
        smach.State.__init__(self, outcomes=['Failed', 'follow'])
        self._robot = robot
        self._operator_name = "operator"
        self._lost_timeout = lost_timeout
        self._lost_distance = lost_distance
        self._face_pos_pub = rospy.Publisher('/%s/follow_operator/operator_detected_face' % robot.robot_name,
                                             geometry_msgs.msg.PointStamped, queue_size=10)

    def execute(self, userdata=None):
        rospy.loginfo("Trying to recover the operator")
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("%s, please look at me while I am looking for you" % self._operator_name, block=False)

        # Wait for the operator and find his/her face
        operator_recovery_timeout = self._lost_timeout
        start_time = rospy.Time.now()

        look_distance = 2.0     #magic number 4
        look_angles = [0.0,          #magic numbers
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
        while (rospy.Time.now() - start_time).to_sec() < operator_recovery_timeout:
            if self.preempt_requested():
                return False

            self._robot.head.look_at_point(head_goals[i])
            i += 1
            if i == len(head_goals):
                 i = 0
            self._robot.head.wait_for_motion_done()

            # raw_detections is a list of Recognitions
            # a recognition contains a CategoricalDistribution
            # a CategoricalDistribution is a list of CategoryProbabilities
            # a CategoryProbability has a label and a float
            raw_detections = self._robot.perception.detect_faces()
            best_detection = self._robot.perception.get_best_face_recognition(raw_detections, "operator")

            rospy.loginfo("best_detection = {}".format(best_detection))
            if best_detection:
                roi = best_detection.roi
                try:
                    operator_pos_kdl = self._robot.perception.project_roi(roi=roi, frame_id="map")
                except Exception as e:
                     rospy.logerr("head.project_roi failed: %s", e)
                     return 'Failed'
                operator_pos_ros = kdl_conversions.kdlVectorStampedToPointStamped(operator_pos_kdl)
                self._face_pos_pub.publish(operator_pos_ros)

                recovered_operator = self._robot.ed.get_closest_laser_entity(radius=self._lost_distance,
                                                                              center_point=operator_pos_kdl)
                if recovered_operator:
                    print "Found one!"
                    self._operator_id = recovered_operator.id
                    print "Recovered operator id: %s" % self._operator_id
                    self._operator = recovered_operator
                    self._robot.speech.speak("There you are! Go ahead, I'll follow you again", block=False)
                    self._robot.head.close()
                    self._time_started = rospy.Time.now()
                    return 'follow'
                else:
                    print "Could not find an entity {} meter near {}".format(self._lost_distance, operator_pos_kdl)

        self._robot.head.close()
        # self._turn_towards_operator()
        rospy.sleep(2.0)
        return 'Failed'

def setup_statemachine(robot):
    sm_top = smach.StateMachine(outcomes=['Done', 'Aborted', 'Failed'])
    sm_top.userdata.operator = None

    with sm_top:
        smach.StateMachine.add('LEARN_OPERATOR', LearnOperator(robot),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Failed': 'Failed'},
                               remapping={'operator_learn_in': 'operator', 'operator_learn_out': 'operator'})

        smach.StateMachine.add('ASK_FINALIZE', AskFinalize(robot),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Done': 'Done'})
        smach.StateMachine.add('RECOVERY', Recovery(robot),
                               transitions={'Failed': 'Failed',
                                            'follow': 'CON_FOLLOW'})

        sm_con = smach.Concurrence(outcomes=['recover_operator', 'ask_finalize', 'keep_following'],
                                   default_outcome='keep_following',
                                   outcome_map={'ask_finalize': {'FOLLOWBREAD': 'no_follow_bread',
                                                                 'TRACK': 'track'},
                                                'recover_operator': {'FOLLOWBREAD': 'no_follow_bread',
                                                                     'TRACK': 'no_track'},
                                                'recover_operator': {'FOLLOWBREAD': 'follow_bread',
                                                                     'TRACK': 'no_track'}},
                                   input_keys=['operator'])

        sm_con.userdata.buffer = collections.deque()
        sm_con.userdata.operator = None

        with sm_con:
            smach.Concurrence.add('TRACK', Track(robot), remapping={'buffer_track_in': 'buffer',
                                                                    'buffer_track_out': 'buffer',
                                                                    'operator_track_in': 'operator'})

            smach.Concurrence.add('FOLLOWBREAD', FollowBread(robot), remapping={'buffer_follow_in': 'buffer',
                                                                           'buffer_follow_out': 'buffer'})



        smach.StateMachine.add('CON_FOLLOW', sm_con,
                               transitions={'recover_operator': 'RECOVERY',
                                            'ask_finalize': 'ASK_FINALIZE',
                                            'keep_following': 'CON_FOLLOW'})

        return sm_top


if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)


# import copy, threading

# Sample code Janno
# This class should rather be named Buffer instead of BreadCrumb: it is only used to pass data. The 'breadcrumb' is
# the list that is kept in the execute function of the Follow state (hence NOT a member of the class)
# class BreadCrumb(object):
#     def __init__(self):
#         self._breadcrumb = []
#         self._lock = threading.Lock()
#
#     def set_data(self, data):
#         # Dit moet waarschijnlijk iets van 'append' worden. Alleen zetten is niet voldoende: het kan zomaar zijn dat ik
#         # meerdere malen een punt toe voeg voordat ik weer uitlees
#         # Bij het 'appenden' kun je ook meteen een distance check doen.
#         with self._lock:
#             self._list = copy.deepcopy(data)
#
#             # Equivalent to
#             # self._lock.acquire()
#             # self._list = copy.deepcopy(data)
#             # self._lock.release()
#
#     def append(self):
#         # ToDo Josja
#         pass
#
#     def get_data(self):
#         with self._lock:
#             result = self._breadcrumb
#             self._breadcrumb = []
#         return result
#
#
# class Track(smach.State):
#     def __init__(self, robot, breadcrumb):
#         self.robot = robot
#         self._breadcrumb = breadcrumb
#
#         # Possible new way:
#         # self._tracking_sub = rospy.Subscriber("bla", bla_msgs.Blaat, self._tracking_callback)
#
#     def execute(self):
#         while True:
#             operator = self.robot.ed.get_person()
#             self._breadcrumb.append(operator)
#
#             # self._breadcrumb.append(self._new_data)
#             # self._new_data = []
#
#             # def _tracking_callback(self, msg):
#             #     self._new_data = msg....
#             #     self._breadcrumb.append(msg.data)
#
#
# class Follow(smach.State):
#     def __init__(self, robot, breadcrumb):
#         self._breadcrumb = breadcrumb
#
#         self._lookat_radius = 1.0  # Something like that
#
#         def execute(self):
#             breadcrumb = []  # Create an empty list at the top of our execute hook: we don't want any state remaining
#
#             while True:
#                 new_list
#                 self._breadcrumb.get_data()
#                 breadcrumb = breadcrumb + new_list
#
#                 # Do smart stuff with breadcrumb
#
#                 # Pitfall: breadcrumb does still contain data when entering this hook the second time. How do we solve this?
#
#                 # ToDo: remove points that have been visited by the robot
#
#
# class FollowMachine(smach.ConcurrentStateMachine):
#     breadcrumb = BreadCrumb()
#
#     with sm:
#         smach.StateMachine.add("track", Track(breadcrumb=breadcrumb))
#         smach.StateMachine.add("Follow", Follow(breadcrumb=breadcrumb))
#
#
#


### new idea::
# class BreadCrumb(object):
#     def __init__(self):
#         self._breadcrumb = collections.deque()
#
#     # def set_data(self, data):
#     #     # Dit moet waarschijnlijk iets van 'append' worden. Alleen zetten is niet voldoende: het kan zomaar zijn dat ik
#     #     # meerdere malen een punt toe voeg voordat ik weer uitlees
#     #     # Bij het 'appenden' kun je ook meteen een distance check doen.
#     #     with self._lock:
#     #         self._list = copy.deepcopy(data)
#     #
#     #         # Equivalent to
#     #         # self._lock.acquire()
#     #         # self._list = copy.deepcopy(data)
#     #         # self._lock.release()
#
#     def append(self, data):
#         # ToDo Josja
#         self._breadcrumb.append(data)
#
#
## note that all the buffervariables are popped and placed in a different variable which can be used for planning
#     def get_data(self):
#         result = []
#         while len(self._breadcrumb) > 0:
#             result.append(self._breadcrumb.pop())  # Pop or popleft?
#         return result


