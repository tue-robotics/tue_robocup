from __future__ import absolute_import

# ROS
import rospy

# TU/e Robotics
from cb_planner_msgs_srvs.msg import *
from robot_skills.arms import PublicArm
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint
from ..util.designators import check_resolve_type, AttrDesignator
from ..util.designators.arm import UnoccupiedArmDesignator


class NavigateToGrasp(NavigateTo):
    """"
    Navigate so that the arm can easily grasp the entity

    :param robot: robot object
    :param entity_designator: designator that resolves to an Ed Entity
    :param arm_designator: which arm to eventually grasp with?
    """
    def __init__(self, robot, entity_designator, arm_designator=None):
        check_resolve_type(entity_designator, Entity)  # Check that the entity_designator resolves to an Entity
        pose_designator = AttrDesignator(entity_designator, 'pose', resolve_type=FrameStamped)

        check_resolve_type(arm_designator, PublicArm)  # Check that the arm_designator resolves to an Arm

        if not arm_designator:
            rospy.logerr('NavigateToGrasp: side should be determined by entity_designator.\
            Please specify left or right, will default to left. This is Deprecated')
            arm_designator = UnoccupiedArmDesignator(self.robot, {})

        super(NavigateToGrasp, self).__init__(robot,
                                              lambda: arms_reach_constraint(pose_designator, arm_designator, look=True),
                                              reset_head=False)
