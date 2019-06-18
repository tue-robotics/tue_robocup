"""
Module contains states to guide an operator to a designated location.
"""

# System
import enum

# ROS
import rospy
import smach
import PyKDL as kdl

# Robot skills
from robot_skills.util.kdl_conversions import VectorStamped
from robot_smach_states.util.designators import EdEntityDesignator

# robot_smach_states.navigation
import navigation
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic


def detect_operator_behind_robot(robot, distance=1.0, radius=0.5):

    # type: (Robot, float, float) -> bool
    """
    Checks if a person is within <radius> of the position <distance> behind the <robot>

    :param robot: (Robot) api object
    :param distance: (float) follow distance
    :param radius: (float) radius around the position at <distance> behind the robot
    :return: (bool) whether an operator was detected
    """
    image_data = robot.perception.get_rgb_depth_caminfo()
    success, found_people_ids = robot.ed.detect_people(*image_data)
    found_people = [robot.ed.get_entity(id_) for id_ in found_people_ids]

    # Assume the operator is around 1.0 m behind the robot
    base_pose = robot.base.get_location()
    expected_person_pos = base_pose.frame * kdl.Vector(-distance, 0.0, 0.0)

    for person in found_people:
        if (person.pose.frame.p - expected_person_pos).Norm() < radius:
            return True
    return False


class GuideMode(enum.Enum):
    REGULAR = 0  # Regular mode
    TOUR_GUIDE = 1  # Tour_guide mode: robot also mentions furniture entities along the way


class ExecutePlanGuidance(smach.State):
    """
    Similar to the "executePlan" smach state. The only difference is that after driving for x meters, "check for 
    operator" is returned.
    """
    def __init__(self, robot, mode=GuideMode.REGULAR):
        # type: (Robot) -> None
        smach.State.__init__(self, outcomes=["arrived", "blocked", "preempted", "lost_operator"])
        self.robot = robot
        self._distance_threshold = 1.0  # Only check if the operator is there once we've drived for this distance
        self._mode = mode
        # self._follow_distance = 1.0  # Operator is expected to follow the robot approximately this distance
        # self._operator_radius_threshold = 0.5  # Operator is expected to be within this radius around the position
        # defined by the follow distance

        # Tour guide attributes
        self._room_ids = []
        self._object_ids = []
        
    def execute(self, userdata=None):

        # Look backwards to have the operator in view
        self.robot.head.look_at_point(VectorStamped(-1.0, 0.0, 1.75, self.robot.base_link_frame))

        rate = rospy.Rate(10.0)  # Loop at 10 Hz
        distance = 0.0
        old_position = self._get_base_position()
        operator_stamp = rospy.Time.now()  # Assume the operator is near if we start here
        while not rospy.is_shutdown():

            if self.preempt_requested():
                self.robot.base.local_planner.cancelCurrentPlan()
                rospy.loginfo("execute: preempt_requested")
                return "preempted"

            status = self.robot.base.local_planner.getStatus()

            if status == "arrived":
                return "arrived"
            elif status == "blocked":
                return "blocked"

            new_position = self._get_base_position()
            distance += (new_position - old_position).Norm()
            old_position = new_position
            if distance > self._distance_threshold:
                rospy.logdebug(
                    "Distance {} exceeds threshold {}, check for operator".format(distance, self._distance_threshold))
                if not self._check_operator():
                    rospy.loginfo("Lost operator while guiding, cancelling plan")
                    self.robot.base.local_planner.cancelCurrentPlan()
                    return "lost_operator"

            rate.sleep()

    def _check_operator(self):
        """
        Checks if the operator is still sufficiently close

        :return: (bool)
        """
        # ToDo: make robust (use time stamp?)
        return detect_operator_behind_robot(self.robot)  # , self._follow_distance, self._operator_radius_threshold)

    def _get_base_position(self):
        # type: () -> kdl.Vector
        """
        Gets the base position as a kdl Vector

        :return: (kdl Vector) with current base position
        """
        frame_stamped = self.robot.base.get_location()
        return frame_stamped.frame.p


class WaitForOperator(smach.State):
    def __init__(self, robot, timeout=10.0):
        # type: (Robot, float) -> None
        """
        Smach state to check if the operator is still following the robot.

        :param robot: (Robot) robot api object
        :param timeout: (float) if the operator has not been detected for this period, "is_lost" will be returned
        """
        smach.State.__init__(self, outcomes=["is_following", "is_lost", "preempted"])
        self._robot = robot
        self._timeout = timeout

    def execute(self, ud):

        self._robot.speech.speak("It seems that we have lost each other. Please stand one meter behind me.")

        rate = rospy.Rate(2.0)
        t_start = rospy.Time.now()
        while not rospy.is_shutdown():

            # Check if the operator is there
            if detect_operator_behind_robot(self._robot):
                self._robot.speech.speak("There you are", block=False)
                return "is_following"

            # Check timeout
            if (rospy.Time.now() - t_start).to_sec() > self._timeout:
                rospy.loginfo("Guide - Wait for operator: timeout {} exceeded".format(self._timeout))
                return "is_lost"

            rate.sleep()

        return "preempted"


class Guide(smach.StateMachine):
    def __init__(self, robot):
        # type: (robot) -> None
        """
        Base Smach state to guide an operator to a designated position

        :param robot: (Robot) robot api object
        """
        smach.StateMachine.__init__(
            self, outcomes=["arrived", "unreachable", "goal_not_defined", "lost_operator", "preempted"])
        self.robot = robot

        with self:
            smach.StateMachine.add("GET_PLAN", navigation.getPlan(self.robot, self.generate_constraint),
                                   transitions={"unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",
                                                "goal_ok": "EXECUTE_PLAN"})

            smach.StateMachine.add("EXECUTE_PLAN", ExecutePlanGuidance(self.robot),
                                   transitions={"arrived": "arrived",
                                                "blocked": "PLAN_BLOCKED",
                                                "preempted": "preempted",
                                                "lost_operator": "WAIT_FOR_OPERATOR"})

            smach.StateMachine.add("WAIT_FOR_OPERATOR", WaitForOperator(self.robot),
                                   transitions={"is_following": "GET_PLAN",
                                                "is_lost": "lost_operator"})

            smach.StateMachine.add("PLAN_BLOCKED", navigation.planBlocked(self.robot),
                                   transitions={"blocked": "GET_PLAN",
                                                "free": "EXECUTE_PLAN"})

    @staticmethod
    def generate_constraint():
        raise NotImplementedError("Inheriting Guide states must implement a generate constraint method, preferably"
                                  "by re-using it from a navigation state.")


class GuideToSymbolic(Guide):
    """ Guidance class to navigate to a semantically annotated goal, e.g., in front of the dinner table.
    """
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator):
        # type: (Robot, dict, EdEntityDesignator) -> None
        """ Constructor

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
        resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
        to compute the orientation constraint.
        """
        super(GuideToSymbolic, self).__init__(robot)
        self._entity_designator_area_name_map = entity_designator_area_name_map
        self._entity_lookat_designator = entity_lookat_designator

    def generate_constraint(self):
        # type: () -> tuple
        """
        Generates the constraint using the generate constraint method of NavigateToSymbolic

        :return: (tuple(PositionConstraint, OrientationConstraint)). If one of the entities does not resolve,
        None is returned.
        """
        return NavigateToSymbolic.generate_constraint(
            self.robot, self._entity_designator_area_name_map, self._entity_lookat_designator)
