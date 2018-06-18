# System
import math

# ROS
from geometry_msgs.msg import *
import rospy

# TU/e Robotics
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from robot_skills.util.kdl_conversions import FrameStamped
from robot_smach_states.navigation import NavigateTo
from robot_smach_states.util.designators import Designator, check_resolve_type


# ----------------------------------------------------------------------------------------------------
class NavigateToPlace(NavigateTo):
    def __init__(self, robot, place_pose_designator, arm_designator=None):
        """Navigate so that the arm can reach the place point
        @param place_pose_designator designator that resolves to a geometry_msgs.msg.PoseStamped
        @param arm which arm to eventually place with?
        """
        super(NavigateToPlace, self).__init__(robot)

        self.robot    = robot
        check_resolve_type(place_pose_designator, FrameStamped) #Check that place_pose_designator actually returns a PoseStamped
        self.place_pose_designator = place_pose_designator

        self.arm_designator = arm_designator
        if not arm_designator:
            rospy.logerr('NavigateToPlace: side should be determined by arm_designator. Please specify left or right, will default to left')
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

        place_fs = self.place_pose_designator.resolve()

        if not place_fs:
            rospy.logerr("No such place_pose")
            return None

        rospy.loginfo("Navigating to place at {0}".format(place_fs).replace('\n', ' '))

        try:
            x = place_fs.frame.p.x()
            y = place_fs.frame.p.y()
        except KeyError, ke:
            rospy.logerr("Could not determine pose: ".format(ke))
            return None

        # Outer radius
        radius -= 0.1
        ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.075)
        # pc = PositionConstraint(constraint=ri+" and "+ro, frame="/map")
        # oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map", angle_offset=angle_offset)
        pc = PositionConstraint(constraint=ri+" and "+ro, frame=place_fs.frame_id)
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame=place_fs.frame_id, angle_offset=angle_offset)

        return pc, oc
