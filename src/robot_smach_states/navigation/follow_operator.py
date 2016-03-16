#!/usr/bin/env python

import smach, rospy, sys
from robot_smach_states.util.startup import startup
import robot_smach_states as states

import threading
import time

import math
from visualization_msgs.msg import Marker

from cb_planner_msgs_srvs.msg import *

from robot_skills.util import transformations, msg_constructors


class FollowOperator(smach.State):
    def __init__(self, robot, ask_follow=True, operator_radius=1, timeout=1.0, start_timeout=10, operator_timeout = 20, distance_threshold = 4.0, lost_timeout = 5, lost_distance = 1.5):
        smach.State.__init__(self, outcomes=["stopped",'lost_operator', "no_operator"])
        self._robot = robot
        self._operator_id = None

        self._at_location = False
        self._first_time_at_location = None
        self._operator_radius = operator_radius
        self._timeout = timeout
        self._start_timeout = start_timeout
        self._operator_timeout = operator_timeout
        self._distance_threshold = distance_threshold
        self._last_pose_stamped = None
        self._time_started = None
        self._ask_follow = ask_follow
        self._lost_timeout = lost_timeout
        self._lost_distance = lost_distance

        self._operator_pub = rospy.Publisher('~operator_position', geometry_msgs.msg.PointStamped, queue_size=10)
        self._plan_marker_pub = rospy.Publisher('/%s/global_planner/visualization/markers/global_plan' % robot.robot_name, Marker, queue_size=10)

    def _register_operator(self):
        start_time = rospy.Time.now()

        self._robot.head.look_at_standing_person()

        operator = None
        while not operator:
            if (rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return False

            if self._ask_follow:
                self._robot.speech.speak("Should I follow you?", block=True)
                answer = self._robot.ears.recognize("(yes|no)", {})

                if answer:
                    if answer.result == "yes":
                        operator = self._robot.ed.get_closest_entity(radius=1, center_point=msg_constructors.PointStamped(x=1.5, y=0, z=1, frame_id="/%s/base_link"%self._robot.robot_name))
                        if not operator:
                            self._robot.speech.speak("Please stand in front of me")
                    elif answer.result == "no":
                        return False
                    else:
                        rospy.sleep(2)
                else:
                    self._robot.speech.speak("Something is wrong with my ears, please take a look!")
                    return False
            else:
                operator = self._robot.ed.get_closest_entity(radius=1, center_point=msg_constructors.PointStamped(x=1.5, y=0, z=1, frame_id="/%s/base_link"%self._robot.robot_name))
                if not operator:
                    rospy.sleep(1)

        # Operator is None?
        print "We have a new operator: %s"%operator.id
        self._robot.speech.speak("Ok, I will follow you!", block=False)
        self._operator_id = operator.id

        self._robot.head.close()

        return True

    def _get_operator(self, operator_id):
        if self._operator_id:
            operator = self._robot.ed.get_entity(id=operator_id)
	    
	    if operator:
	        operator_pos = geometry_msgs.msg.PointStamped()
                operator_pos.header.stamp = rospy.get_rostime()
                operator_pos.header.frame_id = operator.id
                operator_pos.point.x = 0.0;
                operator_pos.point.y = 0.0;
                operator_pos.point.z = 0.0;
                self._operator_pub.publish(operator_pos)

        else:
            operator = None

        return operator

    def _visualize_path(self, path):
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

    def _update_navigation(self, breadcrumbs):
        self._robot.head.cancel_goal()

        operator = breadcrumbs[-1]

        p = PositionConstraint()
        p.constraint = "x^2 + y^2 < %f^2"%self._operator_radius
        p.frame = operator.id

        # We are going to do this dependent on distance to operator

        # Get the point of the operator and the robot in map frame
        r_point = self._robot.base.get_location().pose.position
        o_point = breadcrumbs[0].pose.position

        # Get the distance
        dx = o_point.x - r_point.x
        dy = o_point.y - r_point.y
        length = math.hypot(dx, dy)
        print "Breadcrumb distance: "
        print length

        # Store pose if changed and check timeout
        current_pose_stamped = self._robot.base.get_location()
        if not self._last_pose_stamped:
            self._last_pose_stamped = current_pose_stamped
        else:
            # Compare the pose with the last pose and update if difference is larger than x
            if math.hypot(current_pose_stamped.pose.position.x - self._last_pose_stamped.pose.position.x, current_pose_stamped.pose.position.y - self._last_pose_stamped.pose.position.y) > 0.05:
                # Update the last pose
                print "Last pose stamped (%f,%f) at %f secs"%(self._last_pose_stamped.pose.position.x, self._last_pose_stamped.pose.position.y, self._last_pose_stamped.header.stamp.secs)
                self._last_pose_stamped = current_pose_stamped
            else:
                print "We are standing still :/"

            print "Seconds not moved: %f"%(current_pose_stamped.header.stamp - self._last_pose_stamped.header.stamp).to_sec()
            print "Seconds since start: %f"%(current_pose_stamped.header.stamp - self._time_started).to_sec()
            # Check whether we passe the timeout
            if (current_pose_stamped.header.stamp - self._last_pose_stamped.header.stamp).to_sec() > self._timeout:
                print "We are standing still long enough"
                # Only return True if we exceeded the start timeout
                if (current_pose_stamped.header.stamp - self._time_started).to_sec() > self._start_timeout:
                    print "We passed start timeout"
                    if length < 1.0:
                        print "Distance to goal < 1.0 : %f" % length
                        return True

        plan = None
        if length > self._distance_threshold:
            plan = self._robot.base.global_planner.getPlan(p)
        else:
            res = 0.05
            dx_norm = dx / length
            dy_norm = dy / length
            yaw = math.atan2(dy, dx)

            Z = transformations.euler_z_from_quaternion(self._robot.base.get_location().pose.orientation)
            print "robot yaw: %f"%Z
            print "Desired yaw: %f"%yaw

            plan = []

            start = 0
            end = int( (length - self._operator_radius) / res)

            # TODO: Proper fix for this (maybe a path of only 1m before the operator?)
            if end > 20:
                start = 10

            for i in range(start, end):
                x = r_point.x + i * dx_norm * res
                y = r_point.y + i * dy_norm * res
                plan.append(msg_constructors.PoseStamped(x = x, y = y, z = 0, yaw = yaw))

            # TODO: Visualization marker for path

        if plan:
            # Communicate to local planner
            o = OrientationConstraint()
            o.frame = operator.id
            self._visualize_path(plan)
            self._robot.base.local_planner.setPlan(plan, p, o)

        return False # We are not there

    def execute(self, userdata):
        self._robot.head.close()
        if self._robot.robot_name == "amigo":
            self._robot.torso.send_goal('reset', timeout=4.0)

        if not self._register_operator():
            self._robot.base.local_planner.cancelCurrentPlan()
            return "no_operator"

        self._time_started = rospy.Time.now()

        old_operator = None
        breadcrumbs = []

        while not rospy.is_shutdown():

            # Check if operator present still present
            operator = self._get_operator(self._operator_id)
            breadcrumbs.append(operator)

            if not operator:
                self._robot.speech.speak("I lost the operator, but I still have some breadcrumbs to go after before I stop.", block=False)

            if not breadcrumbs and not operator:
                self._robot.speech.speak("I'm out of breadcrumbs. Where is that operator?", block=False)
                lost_time = rospy.Time.now()
                recovered_operator = None
                while rospy.Time.now() - lost_time < rospy.Duration(self._lost_timeout):
                    # Try to catch up with a close entity
                    recovered_operator = self._robot.ed.get_closest_entity(radius=self._lost_distance, center_point=old_operator.pose.position)
                    if recovered_operator:
                        break
                    rospy.sleep(0.2)

                if not recovered_operator:
                    self._robot.speech.speak("I lost you", block=True)
                    self._robot.base.force_drive(0,0,0,0.5)
                    self._robot.base.local_planner.cancelCurrentPlan()
                    return "lost_operator"
                else:
                    print "\n\nWe recovered the operator!\n\n"
                    self._robot.speech.speak("Still following you!", block=False)
                    self._operator_id = recovered_operator.id
                    operator = recovered_operator

            old_operator = operator

            # Update the navigation and check if we are already there
            if self._update_navigation(breadcrumbs):
                self._robot.base.local_planner.cancelCurrentPlan()
                breadcrumbs.pop(0)
                if not breadcrumbs:
                    return "stopped"

            rospy.sleep(1) # Loop at 1Hz

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
