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
        self._operator_name = "operator_name"

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
                    while num_detections < 2: # 5:
                        if self._robot.perception.learn_person(self._operator_name):
                            self._robot.speech.speak("Succesfully detected you %i times" % (num_detections + 1))
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
        self._period = 0.5          #fix this magic number
        self._operator_pub = rospy.Publisher('/%s/follow_operator/operator_position' % robot.robot_name,
                                             geometry_msgs.msg.PointStamped, queue_size=10)
        self._robot = robot
        self._operator_distance = None
        self._breadcrumb_distance = 0.1
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

            self._robot.speech.speak(
                        "Oh no I lost you for a second, please stay where you are and I will come and find you again!")
            return 'no_track'


class FollowBread(smach.State):
    def __init__(self, robot, operator_radius=1, lookat_radius = 1.2):
        smach.State.__init__(self,
                             outcomes=['follow_bread', 'no_follow_bread'],
                             input_keys=['buffer_follow_in'],
                             output_keys=['buffer_follow_out'])
        self._robot = robot
        self._operator_radius = operator_radius
        self._lookat_radius = lookat_radius

    def execute(self, userdata):
        # print list(userdata.buffer)
        buffer = userdata.buffer_follow_in
        while not buffer:
            rospy.sleep(2)
            print('Shit breaks here!')
        newest_crumb = buffer[0]
        operator = buffer[-1]
        if len(buffer) == 1:
            last_operator = buffer[-1]
        else:
            last_operator = buffer[-2]
        robot_position = self._robot.base.get_location().frame
        while len(buffer) >1 and newest_crumb.distance_to_2d(robot_position.p) < self._lookat_radius + 0.1:
            buffer.popleft()

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

        # self._visualize_plan(ros_plan)
        # self._robot.base.local_planner.setPlan(ros_plan, p, o)
        userdata.buffer_follow_out = buffer
        return 'follow_bread'


class AskFinalize(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['follow', 'Done'])
        self.counter2 = 0

    def execute(self, userdata=None):
        var = raw_input("Am I done following you? Yes/No: ")
        if var == "No":
            return 'follow'
        print "Okidoki, we reached the final destination."
        return 'Done'


class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Failed', 'follow'])

    def execute(self, userdata=None):
        var = raw_input("Did I find you again? Yes/No: ")
        if var == "Yes":
            return 'follow'
        print "Oooh noooo, I give up."
        return 'Failed'


def setup_statemachine(robot):
    sm_top = smach.StateMachine(outcomes=['Done', 'Aborted', 'Failed'])
    sm_top.userdata.operator = None

    with sm_top:
        smach.StateMachine.add('LEARN_OPERATOR', LearnOperator(robot),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Failed': 'Failed'},
                               remapping={'operator_learn_in': 'operator', 'operator_learn_out': 'operator'})

        smach.StateMachine.add('ASK_FINALIZE', AskFinalize(),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Done': 'Done'})
        smach.StateMachine.add('RECOVERY', Recovery(),
                               transitions={'Failed': 'Failed',
                                            'follow': 'CON_FOLLOW'})

        sm_con = smach.Concurrence(outcomes=['recover_operator', 'ask_finalize', 'keep_following'],
                                   default_outcome='keep_following',
                                   outcome_map={'ask_finalize': {'FOLLOWBREAD': 'no_follow_bread',
                                                                 'TRACK': 'track'},
                                                'recover_operator': {'FOLLOWBREAD': 'no_follow_bread',
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


