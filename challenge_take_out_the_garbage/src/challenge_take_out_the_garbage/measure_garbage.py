# ROS
import rospy
import smach

# TU/e
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import robot_smach_states.manipulation as manipulation
from robot_skills.arms import PublicArm

from geometry_msgs.msg import WrenchStamped
import math


class MeasureForce(object):
    """
    Measure the three forces in the arm
    """
    def __init__(self, robot):
        """

        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot

    def get_force(self):
        ft_sensor_topic = '/hero/wrist_wrench/raw'

        force_grab = rospy.wait_for_message(ft_sensor_topic, WrenchStamped)

        force_data_x = force_grab.wrench.force.x
        force_data_y = force_grab.wrench.force.y
        force_data_z = force_grab.wrench.force.z

        return [force_data_x, force_data_y, force_data_z]


class MeasureGarbage(smach.StateMachine):
    """
    Measure the weight of the garbage
    """
    def __init__(self, robot):
        """

        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot

    def execute(self):

        return "succeeded"
