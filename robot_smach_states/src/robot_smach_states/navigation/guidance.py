"""
Module contains states to guide an operator to a designated location.
"""

# ROS
import rospy
import smach
from PyKDL as kdl

# Robot skills
from robot_skills.util.kdl_conversions import VectorStamped

# robot_smach_states.navigation
import navigation
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic


class ExecutePlanGuidance(smach.State):
    """
    Similar to the "executePlan" smach state. The only difference is that after driving for x meters, "check for 
    operator" is returned.
    """
    def __init__(self, robot, distance_threshold=2.0):
        smach.State.__init__(self, outcomes=["arrived", "blocked", "preempted", "check_operator"])
        self.robot = robot
        self._distance_threshold = distance_threshold
        self._follow_distance = 1.0  # Operator is expected to follow the robot approximately this distance
        self._operator_radius_threshold = 0.5  # Operator is expected to be within this radius around the position
        # defined by the follow distance
        
    def execute(self, userdata=None):

        # Look backwards to have the operator in view
        self.robot.head.look_at_point(VectorStamped(-1.0, 0.0, 1.75, self.robot.base_link_frame))

        rate = rospy.Rate(10.0)  # Loop at 10 Hz
        distance = 0.0
        old_position = self._get_base_position()
        operator_stamp = rospy.Time.now()  # Assume the operator is near if we start here
        while not rospy.is_shutdown():

            # ToDo: check if need to check for operator

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
                rospy.loginfo(
                    "Distance {} exceeds threshold {}, check for operator".format(distance, self._distance_threshold))
                self.robot.base.local_planner.cancelCurrentPlan()
                return "check_operator"

            rate.sleep()

    def _check_operator(self):
        """
        Checks if the operator is still sufficiently close

        :return: (bool)
        """
        # ToDo: make robust (use time stamp?)
        image_data = self.robot.perception.get_rgb_depth_caminfo()
        success, found_people_ids = self._robot.ed.detect_people(*image_data)
        found_people = [self.robot.ed.get_entity(id_) for id_ in found_people_ids]

        # Assume the operator is around 1.0 m behind the robot
        base_pose = self.robot.pose.get_location()
        expected_person_pos = base_pose.frame * kdl.Vector(-self._follow_distance, 0.0, 0.0)

        for person in found_people:
            if (person.pose.frame.p - expected_person_pos).Norm() < self._operator_radius_threshold:
                return True
        return False

    def _get_base_position(self):
        """
        Gets the base position as a kdl Vector

        :return: (kdl Vector) with current base position
        """
        frame_stamped = self.robot.base.get_location()
        return frame_stamped.frame.p


class CheckOperator(smach.State):
    def __init__(self, robot):
        """
        Smach state to check if the operator is still following the robot.

        :param robot: (Robot) robot api object
        """
        smach.State.__init__(self, outcomes=["is_following", "is_lost"])
        self._robot = robot

    def execute(self, ud):

        # Remember the start position
        start_pose = self._robot.base.get_location()

        # Rotate 90 degrees
        vth = 1.0
        force_timeout = 1.5
        self._robot.base.force_drive(0.0, 0.0, vth, force_timeout)

        self._robot.speech.speak("Now I'm supposed to check if my operator is still there")

        # Rotate back
        self._robot.base.force_drive(0.0, 0.0, -vth, force_timeout)

        return "is_following"


class Guide(smach.StateMachine):
    def __init__(self, robot):
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
                                                "check_operator": "CHECK_OPERATOR"})

            smach.StateMachine.add("CHECK_OPERATOR", CheckOperator(self.robot),
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
        """
        Generates the constraint using the generate constraint method of NavigateToSymbolic

        :return: (tuple(PositionConstraint, OrientationConstraint)). If one of the entities does not resolve,
        None is returned.
        """
        return NavigateToSymbolic.generate_constraint(
            self.robot, self._entity_designator_area_name_map, self._entity_lookat_designator)



