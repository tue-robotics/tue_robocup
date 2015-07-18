#! /usr/bin/env python

import smach, rospy, sys
from robot_smach_states.util.startup import startup
import robot_smach_states as states

import threading
import time

import math

from cb_planner_msgs_srvs.msg import *

from robot_skills.util import transformations, msg_constructors


class FollowOperator(smach.State):
    def __init__(self, robot, operator_position_constraint = "x^2 + y^2 < 1.0^2", timeout = 3.0, operator_timeout = 20, distance_threshold = 1.5):
        smach.State.__init__(self, outcomes=["stopped",'lost_operator', "no_operator"])
        self._robot = robot
        self._operator_id = None

        self._at_location = False
        self._first_time_at_location = None
        self._operator_position_constraint = operator_position_constraint
        self._timeout = timeout
        self._operator_timeout = operator_timeout
        self._distance_threshold = distance_threshold

    def _register_operator(self):
        start_time = rospy.Time.now()

        operator = None
        while not operator:
            if (rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return False

            self._robot.head.look_at_standing_person()
            self._robot.speech.speak("Should I follow you?", block=True)

            answer = self._robot.ears.recognize("(yes|no)", {})

            self._robot.head.cancel_goal()

            if answer:
                if answer.result == "yes":
                    operator = self._robot.ed.get_closest_entity(radius=1, center_point=msg_constructors.PointStamped(x=1.5, y=0, z=1, frame_id="/%s/base_link"%self._robot.robot_name))
                else:
                    return False
            else:
                self._robot.speech.speak("Something is wrong with my ears, please take a look!")

            rospy.sleep(2)

        print "We have a new operator: %s"%operator.id
        self._robot.speech.speak("I will follow you!", block=False)
        self._operator_id = operator.id

        return True

    def _get_operator(self, operator_id):
        if self._operator_id:
            operator = self._robot.ed.get_entity(id=operator_id)
        else:
            operator = None

        return operator

    def _update_navigation(self, operator):
        self._robot.head.cancel_goal()

        p = PositionConstraint()
        p.constraint = self._operator_position_constraint
        p.frame = operator.id

        # We are going to do this dependent on distance to operator

        # Get the point of the operator and the robot in map frame
        r_point = self._robot.base.get_location().pose.position
        o_point = operator.pose.position

        # Get the distance
        dx = r_point.x - o_point.x
        dy = r_point.y - o_point.y
        length = math.hypot(dx, dy)

        plan = None
        if length > self._distance_threshold:
            plan = self._robot.base.global_planner.getPlan(p)
        else:
            res = 0.05
            dx_norm = dx / length
            dy_norm = dy / length
            yaw = atan2(dy, dx)
            plan = []
            for i in range(0, length / res):
                x = r_point.x + i * dx_norm * res
                y = r_point.y + i * dy_norm * res
                plan.append(msg_constructors.PoseStamped(x = x, y = y, z = 0, yaw = yaw))

        if plan:
            # Check whether we are already there
            if len(plan) <= 3:
                if not self._at_location:
                    self._first_time_at_location = rospy.Time.now()
                self._at_location = True

                print "At location!"

                if (rospy.Time.now() - self._first_time_at_location) > rospy.Duration(self._timeout):
                    return True # We are there
            else:
                self._first_time_at_location = None
                self._at_location = False

            # Communicate to local planner
            o = OrientationConstraint()
            o.frame = operator.id
            self._robot.base.local_planner.setPlan(plan, p, o)

        return False # We are not there

    def execute(self, userdata):
        self._robot.head.cancel_goal()

        self._at_location = False
        self._first_time_at_location = None

        if not self._register_operator():
            self._robot.base.local_planner.cancelCurrentPlan()
            return "no_operator"

        while not rospy.is_shutdown():

            # Check if operator present still present
            operator = self._get_operator(self._operator_id)

            if not operator:
                self._robot.speech.speak("I lost you", block=True)
                self._robot.base.local_planner.cancelCurrentPlan()
                return "lost_operator"

            # Update the navigation and check if we are already there
            if self._update_navigation(operator):
                self._robot.base.local_planner.cancelCurrentPlan()
                return "stopped"

            rospy.sleep(1) # Loop at 1Hz

def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with sm:
        smach.StateMachine.add('TEST', FollowOperator(robot, entity), transitions={"stopped":"TEST",'lost_operator':"TEST", "no_operator":"TEST"})
        return sm

if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)
