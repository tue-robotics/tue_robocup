#!/usr/bin/env python3
import PyKDL as kdl
import rospy
from pykdl_ros import FrameStamped
from ed.shape import RightPrism
from smach import StateMachine, cb_interface, CBState
from ed.entity import Entity
from robocup_knowledge import load_knowledge
from robot_smach_states.utility import Initialize, SetInitialPose
from robot_smach_states.navigation import FollowOperator, NavigateToWaypoint
from robot_smach_states.human_interaction import AskYesNo, Say, GetFurnitureFromOperatorPose
from robot_smach_states.manipulation import Grab, HandoverToHuman, HandoverFromHuman
import robot_smach_states.util.designators as ds
from robot_skills.arm import arms
from robot_smach_states.utility import WaitTime


challenge_knowledge = load_knowledge("challenge_carry_my_luggage")

STARTING_POINT = challenge_knowledge.starting_point


@cb_interface(outcomes=["done"])
def place(userdata, designator, robot):
    entity_id = "bag"
    pose = FrameStamped(
        frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(1.0, 3.0, 0.3)),
        stamp=rospy.Time.now(),
        frame_id="map",
    )
    robot.ed.update_entity(uuid=entity_id, frame_stamped=pose)
    shape = RightPrism(
        [kdl.Vector(0, 0, 0), kdl.Vector(0, 0.05, 0), kdl.Vector(0.05, 0.05, 0), kdl.Vector(0.05, 0, 0)], -0.1, 0.1
    )
    item = Entity(entity_id, "bag", pose.header.frame_id, pose.frame, shape, None, None, rospy.Time.now())
    designator.write(item)
    return "done"


class CarryMyLuggage(StateMachine):
    def __init__(self, robot):
        """

        :param robot:
        """
        StateMachine.__init__(self, outcomes=["Done", "Aborted"])
        self.robot = robot
        self.entity_designator = ds.VariableDesignator(resolve_type=Entity)
        self.arm_designator = ds.UnoccupiedArmDesignator(
            robot,
            {
                "required_goals": ["reset", "handover_to_human", "carrying_pose"],
                "required_gripper_types": [arms.GripperTypes.GRASPING],
            },
        ).lockable()
        self.waypoint_designator = ds.EntityByIdDesignator(robot, uuid=STARTING_POINT)
        # noinspection PyProtectedMember
        self._arm = robot.get_arm()._arm

        with self:
            StateMachine.add(
                "INITIALIZE",
                Initialize(self.robot),
                transitions={"initialized": "SET_INITIAL_POSE", "abort": "SET_INITIAL_POSE"},
            )

            StateMachine.add(
                "SET_INITIAL_POSE",
                SetInitialPose(self.robot, STARTING_POINT),
                transitions={
                    "done": "MOVE_CUSTOM_CARRY",  # Choice here; try to pick up the bag or not: MOVE_CUSTOM_CARRY or POINT_BAG
                    "preempted": "MOVE_CUSTOM_CARRY",  # todo: change this?
                    "error": "MOVE_CUSTOM_CARRY",  # Choice here; try to pick up the bag or not: MOVE_CUSTOM_CARRY or POINT_BAG
                },
            )

            # Choice 1; Do no try to pick up the bag
            @cb_interface(outcomes=["done"])
            def move_to_custom_carry_pose(_):
                p = [0.15, 0, -1.1, -1.37, 0]
                # noinspection PyProtectedMember
                self._arm._send_joint_trajectory([p], timeout=rospy.Duration(0))
                self._arm.wait_for_motion_done()
                self._arm.gripper.send_goal('open')
                self._arm.wait_for_motion_done()
                return "done"

            self.add("MOVE_CUSTOM_CARRY", CBState(move_to_custom_carry_pose), transitions={"done": "ASK_BAG_HANDOVER"})

            StateMachine.add(
                "ASK_BAG_HANDOVER",
                Say(
                    self.robot,
                    ["I am unable to pick up the bag, please put it in my gripper"],
                    block=True,
                    look_at_standing_person=True,
                ),
                transitions={
                    "spoken": "WAIT_FOR_OPERATOR",
                },
            )

            StateMachine.add(
                "WAIT_FOR_OPERATOR",
                WaitTime(15),
                transitions={
                    "waited": "CLOSE_GRIPPER",
                    "preempted": "CLOSE_GRIPPER"
                },
            )

            @cb_interface(outcomes=["done"])
            def gripper_close(_):
                self._arm.gripper.send_goal('close')
                self._arm.wait_for_motion_done()
                return "done"

            self.add("CLOSE_GRIPPER", CBState(gripper_close),
                     transitions={"done": "SAY_BAG_HANDOVER_SUCCESS"})

            # Old behavior with a handover
            # StateMachine.add(
            #     "BAG_HANDOVER_FROM_HUMAN",
            #     HandoverFromHuman(self.robot, self.arm_designator, "bag", None, 15),
            #     transitions={
            #         "succeeded": "SAY_BAG_HANDOVER_SUCCESS",
            #         "failed": "SAY_BAG_HANDOVER_FAILED",
            #         "timeout": "SAY_BAG_HANDOVER_FAILED",
            #     },
            # )
            # StateMachine.add(
            #     "SAY_BAG_HANDOVER_FAILED",
            #     Say(
            #         self.robot,
            #         ["It seems the handing over failed... Let me just accompany you to your car!"],
            #         block=True,
            #         look_at_standing_person=True,
            #     ),
            #     transitions={
            #         "spoken": "FOLLOW_OPERATOR",
            #     },
            # )

            StateMachine.add(
                "SAY_BAG_HANDOVER_SUCCESS",
                Say(
                    self.robot,
                    ["Lets go to your car! I will follow you!"],
                    block=True,
                    look_at_standing_person=True,
                ),
                transitions={
                    "spoken": "FOLLOW_OPERATOR",
                },
            )

            # # Choice 2: Try to pick up the bag
            # StateMachine.add(
            #     "POINT_BAG",
            #     Say(
            #         self.robot,
            #         ["Please point at the bag you want me to carry and await further instructions!"],
            #         block=True,
            #         look_at_standing_person=True,
            #     ),
            #     transitions={
            #         "spoken": "GET_ENTITY_POSE",
            #     },
            # )
            #
            # # workaround to remove dependency from simulating raytracing
            # StateMachine.add(
            #     "GET_ENTITY_POSE",
            #     CBState(place, cb_args=[self.entity_designator.writeable, self.robot]),
            #     transitions={"done": "GRAB_BAG"},
            # )
            #
            # # StateMachine.add(
            # #     'GET_ENTITY_POSE',
            # #     GetFurnitureFromOperatorPose(self.robot, self.entity_designator.writeable),
            # #     transitions={'succeeded': 'GRAB_BAG',
            # #                  'failed': 'GRAB_BAG'} #todo: change this?
            # # )
            #
            # StateMachine.add(
            #     "GRAB_BAG",
            #     Grab(self.robot, self.entity_designator, self.arm_designator),
            #     transitions={"done": "FOLLOW_OPERATOR", "failed": "SAY_BAG_GRAB_FAILED"},
            # )
            #
            # StateMachine.add(
            #     "SAY_BAG_GRAB_FAILED",
            #     Say(
            #         self.robot,
            #         ["I'm unable to grab your bag... Let me just accompany you to your car!"],
            #         block=True,
            #         look_at_standing_person=True,
            #     ),
            #     transitions={
            #         "spoken": "FOLLOW_OPERATOR",  # ToDo: Change this to handover bag handover?
            #     },
            # )

            # End of choices
            StateMachine.add(
                "FOLLOW_OPERATOR",
                FollowOperator(self.robot, operator_timeout=30, ask_follow=True, learn_face=False, replan=True),
                transitions={
                    "stopped": "ASK_FOR_TASK",
                    "lost_operator": "ASK_FOR_TASK",
                    "no_operator": "ASK_FOR_TASK",
                },
            )

            StateMachine.add(
                "ASK_FOR_TASK",
                Say(
                    self.robot,
                    ["Are we at the car already?"],
                    block=True,
                    look_at_standing_person=True,
                ),
                transitions={
                    "spoken": "WAIT_FOR_TASK",
                },
            )

            StateMachine.add(
                "WAIT_FOR_TASK",
                AskYesNo(self.robot),
                transitions={"yes": "HANDOVER_TO_HUMAN", "no": "FOLLOW_OPERATOR", "no_result": "ASK_FOR_TASK"}, #todo this last one can create an infitine loop
            )

            StateMachine.add(
                "HANDOVER_TO_HUMAN",
                HandoverToHuman(self.robot, self.arm_designator),
                transitions={
                    "succeeded": "NAVIGATE_TO_ARENA",
                    "failed": "NAVIGATE_TO_ARENA",  # todo change this?
                },
            )

            StateMachine.add(
                "NAVIGATE_TO_ARENA",
                NavigateToWaypoint(self.robot, self.waypoint_designator),
                transitions={
                    "arrived": "Done",
                    "unreachable": "Done",  # todo change this?
                    "goal_not_defined": "Done",  # todo change this?
                },
            )


if __name__ == "__main__":
    from challenge_carry_my_luggage.carry_my_luggage import CarryMyLuggage
    from robot_skills import get_robot
    import sys
    import rospy

    if len(sys.argv) < 2:
        print("Please provide robot name as argument.")
        sys.exit(1)

    rospy.init_node("carry_my_luggage_exec")

    robot_name = sys.argv[1]
    robot = get_robot(robot_name)

    CarryMyLuggage(robot)
