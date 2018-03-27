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
        self._period = 0.3          #fix this magic number
        self._operator_pub = rospy.Publisher('/%s/follow_operator/operator_position' % robot.robot_name,
                                             geometry_msgs.msg.PointStamped, queue_size=10)
        self._robot = robot
        self._operator_distance = None
        self._breadcrumb_distance = 0.1
        self._last_operator = None
        self._last_operator_id = None
        self._operator = None
        self._lost_operator = None

    def execute(self, userdata):
        if userdata.operator_track_in:
            operator = userdata.operator_track_in
            self._operator = operator
        else:
            operator = self._operator
        buffer = userdata.buffer_track_in

        operator = self._robot.ed.get_entity(id=operator.id )
        if operator:
            if len(buffer) == 8: #and (rospy.Time.now().to_sec() - operator.last_update_time) > self._period:
                self._robot.speech.speak("Not so fast!")
                print len(buffer)
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
                buffer.append(operator)
            userdata.buffer_track_out = buffer
            return 'track'

        else:
            rospy.sleep(2)
            operator = self._robot.ed.get_entity(id=self._last_operator_id)
            if not operator:
                if not self._lost_operator:
                    self._robot.speech.speak("Oh no I lost you for a second, please stay where you are and I will come and find you again!")
                    self._lost_operator = True
                    return 'track'

                self._robot.base.local_planner.cancelCurrentPlan()
                return 'no_track'
            else:
                return 'track'

class FollowBread(smach.State):
    def __init__(self, robot, operator_radius=1, lookat_radius=1.2):
        smach.State.__init__(self,
                             outcomes=['follow_bread', 'no_follow_bread_ask_finalize', 'no_follow_bread_recovery'],
                             input_keys=['buffer_follow_in'],
                             output_keys=['buffer_follow_out'])
        self._robot = robot
        self._operator_radius = operator_radius
        self._lookat_radius = lookat_radius
        #self._breadcrumb_pub = rospy.Publisher('/%s/global_planner/visualization/markers/breadcrumbs' % robot.robot_name, Marker, queue_size=10)

        self._breadcrumb_pub = rospy.Publisher('/%s/follow_operator/breadcrumbs' % robot.robot_name, Marker, queue_size=10)
        #self._plan_marker_pub = rospy.Publisher('/%s/follow_operator/test_path' % robot.robot_name, Marker, queue_size=10)

        self._plan_marker_pub = rospy.Publisher(
                '/%s/global_planner/visualization/markers/global_plan' % robot.robot_name, Marker, queue_size=10)
        self._newest_crumb = None
        self._operator = None
        self._have_followed = False
        self._current_operator = collections.deque()


    def execute(self, userdata):
        operator = None

        buffer = userdata.buffer_follow_in
        if len(buffer) > 5: #5
            self._have_followed = True

        robot_position = self._robot.base.get_location().frame
        if not buffer:
            # if not self._have_followed:
            rospy.sleep(2)      #magic number

        if buffer:
            self._newest_crumb = buffer[0]
            self._operator = buffer[-1]

        if self._operator:
            operator = self._robot.ed.get_entity(id=self._operator.id)
        print len(buffer)
        print self._have_followed
        if operator:
            print operator.distance_to_2d(robot_position.p)
            if len(buffer) == 1 and self._have_followed and operator.distance_to_2d(robot_position.p) < 1.0: # The only crumb is the operator
                self._have_followed = False
                return 'no_follow_bread_ask_finalize'

        if not buffer:
            return 'no_follow_bread_recovery'

        print len(buffer)
        temp_buffer = collections.deque()
        for crumb in buffer:
            if crumb.distance_to_2d(robot_position.p) > self._lookat_radius + 0.1:
                temp_buffer.append(crumb)
        buffer = temp_buffer
        print len(buffer)
        f = self._robot.base.get_location().frame
        robot_position = f.p
        operator_position = self._operator._pose.p

        ''' Define end goal constraint, solely based on the (old) operator position '''
        p = PositionConstraint()
        p.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2" % (operator_position.x(), operator_position.y(),
                                                       self._operator_radius)

        o = OrientationConstraint()
        o.frame = self._operator.id

        ''' Calculate global plan from robot position, through breadcrumbs, to the operator '''
        res = 0.05      ##magic number number 2
        kdl_plan = []
        previous_point = robot_position

        for crumb in buffer:
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
        if len(ros_plan) > 0:
            if not self._robot.base.global_planner.checkPlan(ros_plan):
                print "Breadcrumb plan is blocked, removing blocked points"
                # Go through plan from operator to robot and pick the first unoccupied point as goal point
                ros_plan = [point for point in ros_plan if self._robot.base.global_planner.checkPlan([point])]

        buffer_msg = Marker()
        buffer_msg.type = Marker.POINTS
        buffer_msg.scale.x = 0.05
        buffer_msg.header.stamp = rospy.get_rostime()
        buffer_msg.header.frame_id = "/map"
        buffer_msg.color.a = 1
        buffer_msg.color.r = 1
        buffer_msg.color.g = 0
        buffer_msg.color.b = 0
        #buffer_msg.lifetime = rospy.Time(.0)
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
        self._robot.base.local_planner.setPlan(ros_plan, p, o)
        userdata.buffer_follow_out = buffer
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
            self._robot.speech.speak("Since I did not hear from you I assume we are done!")
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
                                   outcome_map={'ask_finalize': {'FOLLOWBREAD': 'no_follow_bread_ask_finalize',
                                                                 'TRACK': 'track'},
                                                'recover_operator': {'FOLLOWBREAD': 'no_follow_bread_recovery',
                                                                     'TRACK': 'no_track'}},
                                                #'recover_operator': {'FOLLOWBREAD': 'follow_bread',
                                                #                     'TRACK': 'no_track'}
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
                               transitions={'recover_operator': 'RECOVERY', # 'RECOVERY',
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

