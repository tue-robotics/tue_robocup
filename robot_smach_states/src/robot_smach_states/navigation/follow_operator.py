#!/usr/bin/env python

import smach, rospy, sys
from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import VariableDesignator
import robot_smach_states as states

import threading
import time
import itertools
import PyKDL as kdl

import math
from visualization_msgs.msg import Marker

from cb_planner_msgs_srvs.msg import *

from robot_skills.util import transformations, msg_constructors
from robot_skills.util.kdl_conversions import VectorStamped


class FollowOperator(smach.State):
    def __init__(self, robot, ask_follow=True, learn_face=True, operator_radius=1, lookat_radius=1.2, timeout=1.0, start_timeout=10, operator_timeout=20,
                 distance_threshold=None, lost_timeout=5, lost_distance=0.8,
                 operator_id_des=VariableDesignator(resolve_type=str), standing_still_timeout=20, operator_standing_still_timeout=3.0, replan=False):
        smach.State.__init__(self, outcomes=["stopped",'lost_operator', "no_operator"])
        self._robot = robot
        self._time_started = None
        self._operator = None
        self._operator_id = None
        self._operator_name = "operator"
        self._operator_radius = operator_radius
        self._lookat_radius = lookat_radius
        self._start_timeout = start_timeout
        self._breadcrumbs = []
        self._breadcrumb_distance = 0.1  # meters between dropped breadcrumbs
        self._operator_timeout = operator_timeout
        self._ask_follow = ask_follow
        self._learn_face = learn_face
        self._lost_timeout = lost_timeout
        self._lost_distance = lost_distance
        self._standing_still_timeout = standing_still_timeout
        self._operator_standing_still_timeout = operator_standing_still_timeout

        self._operator_id_des = operator_id_des
        self._operator_distance = None

        self._operator_pub = rospy.Publisher('/%s/follow_operator/operator_position' % robot.robot_name, geometry_msgs.msg.PointStamped, queue_size=10)
        self._plan_marker_pub = rospy.Publisher('/%s/global_planner/visualization/markers/global_plan' % robot.robot_name, Marker, queue_size=10)
        self._breadcrumb_pub = rospy.Publisher('/%s/follow_operator/breadcrumbs' % robot.robot_name, Marker, queue_size=10)
        self._face_pos_pub = rospy.Publisher('/%s/follow_operator/operator_detected_face' % robot.robot_name, geometry_msgs.msg.PointStamped, queue_size=10)

        self._last_pose_stamped = None
        self._last_pose_stamped_time = None
        self._last_operator_pose_stamped = None

        self._replan_active = False
        self._last_operator = None
        self._replan_allowed = replan
        self._replan_timeout = 15 # seconds before another replan is allowed
        self._replan_time = rospy.Time.now() - rospy.Duration(self._replan_timeout)
        self._replan_attempts = 0
        self._max_replan_attempts = 3

        self._period = 0.5

    def _operator_standing_still_for_x_seconds(self, timeout):
        if not self._operator:
            return False

        operator_current_pose = self._operator.pose
        operator_current_pose_stamped = msg_constructors.PoseStamped(x=operator_current_pose.position.x, y=operator_current_pose.position.y)
        #print "Operator position: %s" % self._operator.pose.position

        if not self._last_operator_pose_stamped:
            self._last_operator_pose_stamped = operator_current_pose_stamped
        else:
            # Compare the pose with the last pose and update if difference is larger than x
            if math.hypot(operator_current_pose_stamped.pose.position.x - self._last_operator_pose_stamped.pose.position.x, operator_current_pose_stamped.pose.position.y - self._last_operator_pose_stamped.pose.position.y) > 0.15:
                # Update the last pose
           #     print "Last pose stamped operator (%f,%f) at %f secs"%(self._last_operator_pose_stamped.pose.position.x, self._last_operator_pose_stamped.pose.position.y, self._last_operator_pose_stamped.header.stamp.secs)
                self._last_operator_pose_stamped = operator_current_pose_stamped
            else:
                print "Operator is standing still for %f seconds" % (operator_current_pose_stamped.header.stamp - self._last_operator_pose_stamped.header.stamp).to_sec()
                # Check whether we passed the timeout
                if (operator_current_pose_stamped.header.stamp - self._last_operator_pose_stamped.header.stamp).to_sec() > timeout:
                    return True
        return False

    def _standing_still_for_x_seconds(self, timeout):
        current_pose_stamped = self._robot.base.get_location()
        now = rospy.Time.now()

        if not self._last_pose_stamped:
            self._last_pose_stamped = current_pose_stamped
            self._last_pose_stamped_time = now
        else:
            current_yaw = current_pose_stamped.M.GetRPY()[2]  # Get the Yaw
            last_yaw = self._last_pose_stamped.M.GetRPY()[2]  # Get the Yaw

            # Compare the pose with the last pose and update if difference is larger than x
            if kdl.diff(current_pose_stamped.p, self._last_pose_stamped.p).Norm() > 0.05 or abs(current_yaw - last_yaw) > 0.3:
                # Update the last pose
          #      print "Last pose stamped (%f,%f) at %f secs"%(self._last_pose_stamped.pose.position.x, self._last_pose_stamped.pose.position.y, self._last_pose_stamped.header.stamp.secs)
                self._last_pose_stamped = current_pose_stamped
                self._last_pose_stamped_time = rospy.Time.now()
            else:
         #       print "Robot is standing still :/"

                print "Robot dit not move for x seconds: %f"%(now - self._last_pose_stamped_time).to_sec()

                # Check whether we passed the timeout
                if (now - self._last_pose_stamped_time).to_sec() > timeout:
                    return True
        return False

    def _register_operator(self):
        start_time = rospy.Time.now()

        self._robot.head.look_at_standing_person()

        if self._operator_id:
            operator = self._robot.ed.get_entity( id=self._operator_id )
        else:
            operator = None

        while not operator:
            if (rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return False

            if self._ask_follow:
                self._robot.speech.speak("Should I follow you?", block=True)
                answer = self._robot.ears.recognize("<choice>", {"choice" : ["yes", "no"]})

                if answer and 'choice' in answer.choices:
                    if answer.choices['choice'] == "yes":
                        operator = self._robot.ed.get_closest_laser_entity(radius=0.5, center_point=msg_constructors.PointStamped(x=1.0, y=0, z=1, frame_id="/%s/base_link"%self._robot.robot_name))

                        if not operator:
                            self._robot.speech.speak("Please stand in front of me")
                        else:
                            if self._learn_face:
                                self._robot.speech.speak("Please look at me while I learn to recognize you.", block=True)
                                self._robot.speech.speak("Just in case...",block=False)
                                self._robot.head.look_at_standing_person()
                                learn_person_start_time = rospy.Time.now()
                                learn_person_timeout = 10.0 # TODO: Parameterize
                                num_detections = 0
                                while num_detections < 5:
                                    rospy.logerr("self._robot.ed.learn _person(self._operator_name) method disappeared!, returning False")
                                    if False:
                                        num_detections+=1
                                    elif (rospy.Time.now() - learn_person_start_time).to_sec() > learn_person_timeout:
                                        self._robot.speech.speak("Please stand in front of me and look at me")
                                        operator = None
                                        break

                    elif answer.choices['choice'] == "no":
                        return False
                    else:
                        rospy.sleep(2)
                else:
                    self._robot.speech.speak("Something is wrong with my ears, please take a look!")
                    return False
            else:
                operator = self._robot.ed.get_closest_possible_person_entity(radius=1, center_point=msg_constructors.PointStamped(x=1.5, y=0, z=1, frame_id="/%s/base_link"%self._robot.robot_name))
                if not operator:
                    rospy.sleep(1)

        print "We have a new operator: %s"%operator.id
        self._robot.speech.speak("Gotcha! I will follow you!", block=False)
        self._operator_id = operator.id
        self._operator = operator
        self._breadcrumbs.append(operator)

        self._robot.head.close()

        print ("NOW!!!")
        rospy.sleep(3)

        return True

    def _update_breadcrumb_path(self):
        ''' If the last breadcrumb is less than a threshold away, replace
        the last breadcrumb with the latest operator position; otherwise
        just add it. '''
        if self._operator_id:
            if self._breadcrumbs:
                if self._breadcrumbs[-1].distance_to_2d(self._operator._pose.p) < self._breadcrumb_distance:
                    self._breadcrumbs[-1] = self._operator
                else:
                    self._breadcrumbs.append(self._operator)
            else:
                self._breadcrumbs.append(self._operator)

        # Remove 'reached' breadcrumbs from breadcrumb path
        robot_position = self._robot.base.get_location()
        # robot_yaw = transformations.euler_z_from_quaternion(self._robot.base.pose.orientation)
        temp_crumbs = []
        for crumb in self._breadcrumbs:
            if crumb.distance_to_2d(robot_position.p) > self._lookat_radius + 0.1:
                temp_crumbs.append(crumb)
            else:
                temp_crumbs = []

        self._breadcrumbs = temp_crumbs

        self._visualize_breadcrumbs()

    def _backup_register(self):
        # This only happens when the operator was just registered, and never tracked
        print "Operator already lost. Getting closest possible person entity at 1.5 m in front, radius = 1"
        self._operator = self._robot.ed.get_closest_possible_person_entity(radius=1,
                                                                                center_point=msg_constructors.PointStamped(
                                                                                    x=1.5, y=0, z=1,
                                                                                    frame_id="/%s/base_link" % self._robot.robot_name))
        if self._operator:
            return True
        else:
            print "Operator still lost. Getting closest possible laser entity at 1.5 m in front, radius = 1"
            self._operator = self._robot.ed.get_closest_laser_entity(radius=1,
                                                                          center_point=msg_constructors.PointStamped(
                                                                              x=1.5, y=0, z=1,
                                                                              frame_id="/%s/base_link" % self._robot.robot_name)
                                                                          )

        if self._operator:
            return True
        else:
            print "Trying to register operator again"
            self._robot.speech.speak("Oops, let's try this again...", block=False)
            self._register_operator()
            self._operator = self._robot.ed.get_entity( id=self._operator_id )

        if self._operator:
            self._last_operator = self._operator
            return True

        return False

    def _track_operator(self):
        if self._operator_id:
            self._operator = self._robot.ed.get_entity( id=self._operator_id )
        else:
            self._operator = None

        if self._operator:
            if (rospy.Time.now().to_sec() - self._operator.last_update_time) > self._period:
                self._robot.speech.speak("Not so fast!")

            # If the operator is still tracked, it is also the last_operator
            self._last_operator = self._operator

            operator_pos = geometry_msgs.msg.PointStamped()
            operator_pos.header.stamp = rospy.get_rostime()
            operator_pos.header.frame_id = self._operator_id
            operator_pos.point.x = 0.0
            operator_pos.point.y = 0.0
            operator_pos.point.z = 0.0
            self._operator_pub.publish(operator_pos)

            self._operator_distance = (self._last_operator.distance_to_2d(self._robot.base.get_location().p))

            return True
        else:
            robot_position = self._robot.base.get_location().pose.position

            if not self._last_operator:
                if self._backup_register():
                    # If the operator is still tracked, it is also the last_operator
                    self._last_operator = self._operator

                    operator_pos = geometry_msgs.msg.PointStamped()
                    operator_pos.header.stamp = rospy.get_rostime()
                    operator_pos.header.frame_id = self._operator_id
                    operator_pos.point.x = 0.0
                    operator_pos.point.y = 0.0
                    operator_pos.point.z = 0.0
                    self._operator_pub.publish(operator_pos)

                    self._operator_distance = (self._last_operator.distance_to_2d(self._robot.base.get_location().p))

                    return True
                else:
                    self._robot.speech.speak("I'm sorry, but I couldn't find a person to track")

            self._operator_distance = (self._last_operator.distance_to_2d(self._robot.base.get_location().p))
            # If the operator is lost, check if we still have an ID
            if self._operator_id:
                # At the moment when the operator is lost, tell him to slow down and clear operator ID
                self._operator_id = None
                self._robot.speech.speak("Stop! I lost you! Until I find you again, please wait there.", block=False)
            return False

    def _visualize_breadcrumbs(self):
        breadcrumbs_msg = Marker()
        breadcrumbs_msg.type = Marker.POINTS
        breadcrumbs_msg.scale.x = 0.05
        breadcrumbs_msg.header.stamp = rospy.get_rostime()
        breadcrumbs_msg.header.frame_id = "/map"
        breadcrumbs_msg.color.a = 1
        breadcrumbs_msg.color.r = 0
        breadcrumbs_msg.color.g = 1
        breadcrumbs_msg.color.b = 1
        breadcrumbs_msg.lifetime = rospy.Time(1.0)
        breadcrumbs_msg.id = 0
        breadcrumbs_msg.action = Marker.ADD

        for crumb in self._breadcrumbs:
            breadcrumbs_msg.points.append(crumb.pose.position)

        self._breadcrumb_pub.publish(breadcrumbs_msg)

    def _visualize_plan(self, path):
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
        for pose_stamped in path:
            line_strip.points.append(pose_stamped.pose.position)

        self._plan_marker_pub.publish(line_strip)

    def _update_navigation(self):
        self._robot.head.cancel_goal()

        robot_position = self._robot.base.get_location().p
        operator_position = self._last_operator._pose.p

        ''' Define end goal constraint, solely based on the (old) operator position '''
        p = PositionConstraint()
        p.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2"% (operator_position.x(), operator_position.y(), self._operator_radius)

        o = OrientationConstraint()
        if self._operator_id:
            o.frame = self._operator_id
        else:
            o.frame = 'map'
            o.look_at = self._last_operator.pose.position

        ''' Calculate global plan from robot position, through breadcrumbs, to the operator '''
        res = 0.05
        plan = []
        previous_point = robot_position

        if self._operator:
            breadcrumbs = self._breadcrumbs + [self._operator]
        else:
            breadcrumbs = self._breadcrumbs + [self._last_operator]
        for crumb in breadcrumbs:
            dx = crumb.pose.position.x - previous_point.x
            dy = crumb.pose.position.y - previous_point.y

            length = crumb.distance_to_2d(previous_point)

            if length != 0:
                dx_norm = dx / length
                dy_norm = dy / length
                yaw = math.atan2(dy, dx)

                start = 0
                end = int(length / res)

                for i in range(start, end):
                    x = previous_point.x + i * dx_norm * res
                    y = previous_point.y + i * dy_norm * res
                    plan.append(msg_constructors.PoseStamped(x=x, y=y, z=0, yaw=yaw))

            previous_point = crumb.pose.position

        # Delete the elements from the plan within the operator radius from the robot
        cutoff = int(self._operator_radius/(2.0*res))
        if len(plan) > cutoff:
            del plan[-cutoff:]

        # Check if plan is valid. If not, remove invalid points from the path
        if not self._robot.base.global_planner.checkPlan(plan):
            print "Breadcrumb plan is blocked, removing blocked points"
            # Go through plan from operator to robot and pick the first unoccupied point as goal point
            plan = [point for point in plan if self._robot.base.global_planner.checkPlan([point])]

        self._visualize_plan(plan)
        self._robot.base.local_planner.setPlan(plan, p, o)

    def _recover_operator(self):
        print "Trying to recover the operator"
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("%s, please look at me while I am looking for you" % self._operator_name, block=False)

        # Wait for the operator and find his/her face
        operator_recovery_timeout = 60.0 #TODO: parameterize
        start_time = rospy.Time.now()
        recovered_operator = None

        look_distance = 2.0
        look_angles = [0.0,
                       math.pi/6,
                       math.pi/4,
                       math.pi/2.3,
                       0.0,
                       -math.pi/6,
                       -math.pi/4,
                       -math.pi/2.3]
        head_goals = [VectorStamped(x=look_distance*math.cos(angle),
                                    y=look_distance*math.sin(angle),
                                    z=1.7,
                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in look_angles
                      ]

        i = 0
        while (rospy.Time.now() - start_time).to_sec() < operator_recovery_timeout:
            self._robot.head.look_at_point(head_goals[i])
            i += 1
            if i == len(head_goals):
                i = 0

            self._robot.head.wait_for_motion_done()
            print "Trying to detect faces..."
            rospy.logerr("ed.detect _persons() method disappeared! This was only calling the face recognition module and we are using a new one now!")
            rospy.logerr("I will return an empty detection list!")
            detections = []
            if not detections:
                detections = []
            best_score = -0.5 # TODO: magic number
            best_detection = None
            for d in detections:
                print "name: %s" % d.name
                print "score: %f" % d.name_score
                if d.name == self._operator_name and d.name_score > best_score:
                    best_score = d.name_score
                    best_detection = d

                if not d.name:
                    best_detection = None
                    break

            if best_detection:
                print "Trying to find closest laser entity to face"
                print "best detection frame id: %s"%best_detection.pose.header.frame_id
                operator_pos = geometry_msgs.msg.PointStamped()
                operator_pos.header.stamp = best_detection.pose.header.stamp
                operator_pos.header.frame_id = best_detection.pose.header.frame_id
                operator_pos.point = best_detection.pose.pose.position
                self._face_pos_pub.publish(operator_pos)

                recovered_operator = self._robot.ed.get_closest_possible_person_entity(radius=self._lost_distance,
                                                                             center_point=best_detection.pose.pose.position)

                if not recovered_operator:
                    recovered_operator = self._robot.ed.get_closest_laser_entity(radius=self._lost_distance,
                                                                             center_point=best_detection.pose.pose.position)

            if recovered_operator:
                print "Found one!"
                self._operator_id = recovered_operator.id
                print "Recovered operator id: %s" % self._operator_id
                self._operator = recovered_operator
                self._robot.speech.speak("There you are! Go ahead, I'll follow you again",block=False)
                self._robot.head.close()
                self._time_started = rospy.Time.now()
                return True

        self._robot.head.close()
        self._turn_towards_operator()
        self._update_navigation()
        rospy.sleep(2.0)
        return False

    def _turn_towards_operator(self):
        robot_position = self._robot.base.get_location().p
        operator_position = self._last_operator.pose.position

        p = PositionConstraint()
        p.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2"% (operator_position.x, operator_position.y, self._operator_radius)

        o = OrientationConstraint()
        if self._operator_id:
            o.frame = self._operator_id
        else:
            o.frame = 'map'
            o.look_at = self._last_operator.pose.position

        dx = operator_position.x - robot_position.x
        dy = operator_position.y - robot_position.y

        yaw = math.atan2(dy, dx)
        plan = [msg_constructors.PoseStamped(x=robot_position.x, y=robot_position.y, z=0, yaw=yaw)]
        print "Operator within self._lookat_radius"

        self._robot.base.local_planner.setPlan(plan, p, o)

    def _replan(self):
        self._replan_attempts += 1
        print "Trying to get a global plan"
        operator_position = self._last_operator.pose.position
        # Define end goal constraint, solely based on the (old) operator position
        self._replan_pc = PositionConstraint()
        self._replan_pc.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2" % (operator_position.x, operator_position.y, self._operator_radius)
        plan = self._robot.base.global_planner.getPlan(self._replan_pc)
        if not plan or not self._robot.base.global_planner.checkPlan(plan):
            print "No global plan possible"
        else:
            self._robot.speech.speak("Just a sec, let me try this way.")
            print "Found a global plan, sending it to local planner"
            self._replan_time = rospy.Time.now()
            self._replan_active = True
            oc = self._robot.base.local_planner.getCurrentOrientationConstraint()
            self._visualize_plan(plan)
            self._robot.base.local_planner.setPlan(plan, self._replan_pc, oc)
            self._breadcrumbs = []

    def _check_end_criteria(self):
        # Check if we still have an operator
        lost_operator = self._operator is None

        print "Checking end criteria"

        if self._replan_active:
            if len(self._robot.base.global_planner.getPlan(self._replan_pc)) < 10:
                self._replan_active = False
                if lost_operator and not self._recover_operator():
                    return "lost_operator"

        # Try to recover operator if lost and reached last seen operator position
        print "Operator is at %f meters distance" % self._operator_distance
        if lost_operator and self._operator_distance < self._lookat_radius and self._standing_still_for_x_seconds(1.0): # TODO: HACK! Magic number!
            print "lost operator and within lookat radius and standing still for 1 second"
            if not self._recover_operator():
                self._robot.base.local_planner.cancelCurrentPlan()
                self._robot.speech.speak("I am unable to recover you")
                return "lost_operator"

        # Check are standing still long
        if self._standing_still_for_x_seconds(self._standing_still_timeout):
            # Navigation stuck! One of the following possiblities
            # - Following an operator, operator is still correct, corner is cut or path is otherwise invalid: (path should not have been cut off) replan with global planner and wait for the local planner to get us out of here
            # - Following an operator, operator is still correct, local planner is in local minimum: wait for the local planner to get us out of here (at least 10 s)
            # - Following an operator, operator is not correct, 'operator' is unreachable: try a global plan and wait for the local planner to get us out of here
            # - Not following an operator, planner is in local minimum: try a global plan and wait for the local planner to get us out of here
            self._robot.base.local_planner.cancelCurrentPlan()
            if self._replan_allowed:
                if self._replan_attempts < self._max_replan_attempts:
                    if (rospy.Time.now() - self._replan_time).to_sec() > self._replan_timeout:
                        self._replan()
                else:
                    if not self._recover_operator():
                        return "lost_operator"
            elif not self._recover_operator():
                return "lost_operator"


            #if not self._recover_operator():
             #   self._robot.base.local_planner.cancelCurrentPlan()
             #   self._robot.speech.speak("I am unable to recover you")
             #   return "lost_operator"
        else:
            self._replan_attempts = 0

        # Check if we are already there (in operator radius and operator standing still long enough)
        print "Checking if done following"
        if self._operator_distance < self._operator_radius and self._operator_standing_still_for_x_seconds(self._operator_standing_still_timeout):
            print "I'm close enough to the operator and he's been standing there for long enough"
            print "Checking if we pass the start timeout"
            if (rospy.Time.now() - self._time_started).to_sec() > self._start_timeout:
                print "Passed"
                self._operator_id_des.writeable.write(self._operator_id)
                self._robot.base.local_planner.cancelCurrentPlan()
                return "stopped"
            else:
                print "Not passed"
        else:
            print "apparently not"

        # No end criteria met
        return None

    def execute(self, userdata):
        # Reset robot and operator last pose
        self._last_pose_stamped = None
        self._last_operator_pose_stamped = None
        self._breadcrumbs = []
        old_no_breadcrumbs = len(self._breadcrumbs)

        if self._operator_id_des:
            operator_id = self._operator_id_des.resolve()
            if operator_id:
                self._operator_id = operator_id

        self._robot.head.close()

        if self._robot.robot_name == "amigo":
            self._robot.torso.send_goal('reset', timeout=4.0)

        if not self._register_operator():
            self._robot.base.local_planner.cancelCurrentPlan()
            return "no_operator"

        self._time_started = rospy.Time.now()

        while not rospy.is_shutdown():

            # 1) Track operator
            self._track_operator()

            # 2) Keep track of operator history
            self._update_breadcrumb_path()

            # 3) Check end criteria
            result = self._check_end_criteria()
            if result:
                return result

            # 4) Action
            if not self._operator_standing_still_for_x_seconds(self._operator_standing_still_timeout) and self._operator_distance < self._lookat_radius:
                self._turn_towards_operator()
            else:
                # Only update navigation if there is something to update: operator must have moved
                # if len(self._breadcrumbs) > old_no_breadcrumbs:
                if self._replan_allowed:
                    # If replanned: if recently replanned, only update navigation if not standing still for too long
                    # (to make sure that local planner reaches align state) or just started following
                    print "Replan=True, so check if we replanned..."
                    if self._replan_time.to_sec() > self._time_started.to_sec():
                        print "We did replan at least once"
                        if self._replan_active:
                            print "and this plan is still active, so I'll give the global planner a chance"
                        else:
                            print "but we reached that goal at some point, so we can safely update navigation"
                            self._update_navigation()
                    else:
                        print "We never replanned so far, so we can safely update navigation"
                        self._update_navigation()
                    # else:
                    #     print "Updating navigation"
                    #     self._update_navigation()
                else:
                    self._update_navigation()
                    print "Updating navigation."

            rospy.sleep(self._period) # Loop at 2Hz

def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with sm:
        smach.StateMachine.add('TEST', FollowOperator(robot), transitions={"stopped":"TEST",'lost_operator':"TEST", "no_operator":"TEST"})
        return sm

if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)
