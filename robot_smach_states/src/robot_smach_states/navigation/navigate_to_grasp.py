# System
import math

# ROS
from geometry_msgs.msg import *
import rospy

# TU/e Robotics
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from robot_skills.arms import Arm
from robot_skills.util.entity import Entity
from robot_smach_states.navigation import NavigateTo
from robot_smach_states.util.designators import Designator, check_resolve_type


# ----------------------------------------------------------------------------------------------------
class NavigateToGrasp(NavigateTo):
    def __init__(self, robot, entity_designator, arm_designator=None):
        super(NavigateToGrasp, self).__init__(robot, reset_head=False)

        self.robot    = robot
        check_resolve_type(entity_designator, Entity)  # Check that the entity_designator resolves to an Entity
        self.entity_designator = entity_designator

        check_resolve_type(arm_designator, Arm) #Check that the arm_designator resolves to an Arm
        self.arm_designator = arm_designator
        if not arm_designator:
            rospy.logerr('NavigateToGrasp: side should be determined by entity_designator. Please specify left or right, will default to left')
            self.arm_designator = Designator(robot.leftArm)

    def generateConstraint(self):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return None

        if arm == self.robot.arms['left']:
            angle_offset = math.atan2(-self.robot.grasp_offset.y, self.robot.grasp_offset.x)
        elif arm == self.robot.arms['right']:
            angle_offset = math.atan2(self.robot.grasp_offset.y, self.robot.grasp_offset.x)
        radius = math.hypot(self.robot.grasp_offset.x, self.robot.grasp_offset.y)

        entity = self.entity_designator.resolve()

        if not entity:
            rospy.logerr("No such entity")
            return None

        rospy.loginfo("Navigating to grasp entity id:{0}".format(entity.id))

        try:
            pose = entity.pose #TODO Janno: Not all entities have pose information
            x = pose.frame.p.x()
            y = pose.frame.p.y()
        except KeyError, ke:
            rospy.logerr("Could not determine pose: ".format(ke))
            return None

        try:
            rz, _, _ = entity.pose.frame.M.GetEulerZYX()
        except KeyError, ke:
            rospy.logerr("Could not determine pose.rz: ".format(ke))
            rz = 0

        # Outer radius
        radius -= 0.1
        ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.075)
        pc = PositionConstraint(constraint=ri+" and "+ro, frame="/map")
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map", angle_offset=angle_offset)

        return pc, oc
