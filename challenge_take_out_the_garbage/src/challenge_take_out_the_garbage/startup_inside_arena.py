from __future__ import absolute_import

# ROS
import smach

# TU/e Robotics
from robot_skills.robot import Robot
from robot_smach_states import check_ebutton, utility


class StartChallengeRobustInsideArena(smach.StateMachine):
    """
    Initialize the challenge without waiting for a door to open
    """

    def __init__(self, robot: Robot, initial_pose: str):
        """
        Initialization method for inside the robocup arena

        :param robot: robot object
        :param initial_pose: Identifies the (waypoint) entity to be used as initial pose. For testing purposes,
            a tuple(float, float, float) representing x, y and yaw in map frame can be used.
        """
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        assert hasattr(robot, "base")
        assert hasattr(robot, "speech")

        with self:
            smach.StateMachine.add("NOTIFY_EBUTTON",
                                   check_ebutton.NotifyEButton(robot),
                                   transitions={"succeeded": "INITIALIZE"})

            smach.StateMachine.add("INITIALIZE",
                                   utility.Initialize(robot),
                                   transitions={"initialized": "INIT_POSE",
                                                "abort": "Aborted"})

            smach.StateMachine.add('INIT_POSE',
                                   utility.SetInitialPose(robot, initial_pose),
                                   transitions={'done': 'Done',
                                                'preempted': 'Aborted',
                                                # This transition will never happen at the moment.
                                                # It should never go to aborted.
                                                'error': 'Done'})
