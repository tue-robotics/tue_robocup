# ROS
import smach
import rospy

# TU/e
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills import arms
from robocup_knowledge import load_knowledge
CHALLENGE_KNOWLEDGE = load_knowledge('challenge_take_out_the_garbage')


class DropTrash(smach.State):
    """
    State that moves the robot to the drop bag position and opens the gripper
    """
    def __init__(self, robot, arm_designator):
        """

        :param robot: robot object
        """

        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._arm_designator = arm_designator

    def execute(self, userdata=None):
        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Torso up (non-blocking)
        self._robot.torso.reset()

        # Arm to position in a safe way
        arm.send_joint_goal('handover')
        arm.wait_for_motion_done()
        arm.send_gripper_goal('open')
        arm.wait_for_motion_done()
        arm._arm._send_joint_trajectory(
            [[0.4, -1.0, 0.0, -1.0, 0.0],[0.4, -1.0, 0.0, -1.57, 0.0], [0.4, -1.0, 0.0, -1.0, 0.0],
             [0.4, -1.0, 0.0, -1.57, 0.0]])
        arm.wait_for_motion_done()
        arm.send_joint_goal('reset')
        arm.wait_for_motion_done()
        # arm.send_gripper_goal('close')
        # arm.wait_for_motion_done()
        return "succeeded"


# class DropPoseDesignator(ds.Designator):
#     """
#     Designator to resolver the drop pose
#     """
#     def __init__(self, robot, drop_height, name):
#         """
#         :param robot: robot object
#         :param drop_height: height to drop the object from
#         :param name: name of pose
#         """
#         super(DropPoseDesignator, self).__init__(resolve_type=FrameStamped, name=name)
#
#         self._robot = robot
#         self._drop_height = drop_height
#         self._name = name
#
#     def _resolve(self):
#         frame = None
#
#         # Query ed
#         try:
#             frame = self._robot.ed.get_entity(id=self._name)._pose
#         except:
#             rospy.logwarn("The provided entity has no _pose")
#             return None
#
#         frame.p.z(self._drop_height)
#
#         return FrameStamped(frame, "/map")


class DropDownTrash(smach.StateMachine):
    """
    State that makes the robot go to the drop zone and drop the trash bag there
    """
    def __init__(self, robot, drop_zone_id):
        """
        :param robot: robot object
        :param drop_designator: EdEntityDesignator designating the collection zone
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        arm_designator = ds.OccupiedArmDesignator(
            robot=robot,
            arm_properties={"required_goals": ["handover", "reset", "handover_to_human"],
                            "required_gripper_types": [arms.GripperTypes.GRASPING]})

        with self:
            smach.StateMachine.add("GO_TO_COLLECTION_ZONE",
                                   states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=drop_zone_id),
                                                             radius=0.5),

                                   transitions={"arrived": "DROP_TRASH",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "OPEN_DOOR_PLEASE"})

            smach.StateMachine.add("OPEN_DOOR_PLEASE",
                                   states.Say(robot, "Can you please open the door for me? It seems blocked!"),
                                   transitions={"spoken": "WAIT_FOR_DOOR_OPEN"})

            smach.StateMachine.add("WAIT_FOR_DOOR_OPEN",
                                   states.WaitTime(robot=robot, waittime=5),
                                   transitions={"waited": "GO_TO_COLLECTION_ZONE2",
                                                "preempted": "GO_TO_COLLECTION_ZONE2"})

            smach.StateMachine.add("GO_TO_COLLECTION_ZONE2",
                                   states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=drop_zone_id),
                                                             radius=0.5),

                                   transitions={"arrived": "DROP_TRASH",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            smach.StateMachine.add("DROP_TRASH", DropTrash(robot=robot, arm_designator=arm_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "HANDOVER"})

            smach.StateMachine.add("HANDOVER",
                                   states.HandoverToHuman(robot=robot, arm_designator=arm_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})


#states.NavigateToObserve(robot, drop_designator),
#states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=INTERMEDIATE_1),
#                                                         radius=0.5),
