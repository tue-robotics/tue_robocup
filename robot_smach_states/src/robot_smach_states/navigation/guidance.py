"""
Module contains states to guide an operator to a designated location.
"""

# ROS
import rospy
import smach

# robot_smach_states.navigation
import navigation
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic


class ExecutePlanGuidance(smach.State):
    """
    Similar to the "executePlan" smach state. The only difference is that after driving for x meters, "check for 
    operator" is returned.
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["arrived", "blocked", "preempted", "check_operator"])
        self.robot = robot
        self.t_last_free = None
        
    def execute(self, userdata=None):

        self.t_last_free = rospy.Time.now()

        # Cancel head goal, we need it for navigation :)
        self.robot.head.close()

        rate = rospy.Rate(10.0)  # Loop at 10 Hz
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

            rate.sleep()


class CheckOperator(smach.State):
    def __init__(self, robot):
        """
        Smach state to check if the operator is still following the robot.

        :param robot: (Robot) robot api object
        """
        smach.State.__init__(self, outcomes=["is_following", "is_lost"])
        self._robot = robot

    def execute(self, ud):
        self._robot.speech.speak("Now I'm supposed to check if my operator is still there")
        # ToDo: do something useful
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
                                                "check_operator": "lost_operator"})  # ToDo: update

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


if __name__ == "__main__":

    # Example code
    # (imports placed here because these are only relevant here)
    import rospy
    import sys
    import robot_smach_states.util.designators as ds
    from robot_skills.get_robot import get_robot_from_argv

    assert len(sys.argv) == 3, "Please provide the robot name and the entity id of the object to guide to," \
                               "e.g., 'python guidance.py amigo bed'"

    rospy.init_node("test_guidance")
    r = get_robot_from_argv(1)
    e_id = sys.argv[2]

    s = GuideToSymbolic(r,
                        {ds.EntityByIdDesignator(r, id=e_id): "in_front_of"},
                        ds.EntityByIdDesignator(r, id=e_id)
                        )
    s.execute()
