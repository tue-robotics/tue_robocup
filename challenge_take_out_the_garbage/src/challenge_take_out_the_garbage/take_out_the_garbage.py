# ROS
import smach
import rospy

# Robot smach states
import robot_smach_states as states
from robocup_knowledge import load_knowledge
import robot_smach_states.util.designators as ds
from robot_skills import arms
from challenge_take_out_the_garbage.pick_up import PickUpTrash
from challenge_take_out_the_garbage.drop_down import DropDownTrash, DropTrash
CHALLENGE_KNOWLEDGE = load_knowledge('challenge_take_out_the_garbage')


class TakeOutGarbage(smach.StateMachine):
    """
    Takes the garbage out of two trash bins and take it to the drop zone
    """
    def __init__(self, robot):
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # Create designators
        trashbin_designator = ds.EdEntityDesignator(robot=robot,
                                                    id=CHALLENGE_KNOWLEDGE.trashbin_id,
                                                    name='trashbin_designator')

        # Look if there is a second trash bin present
        # trashbin_designator2 = None
        if hasattr(CHALLENGE_KNOWLEDGE, "trashbin_id2"):
            trashbin_designator2 = ds.EdEntityDesignator(robot=robot,
                                                         id=CHALLENGE_KNOWLEDGE.trashbin_id2,
                                                         name='trashbin_designator2')
            next_state = "HELPER_WAYPOINT"
            rospy.loginfo("There is a second trash bin")
        else:
            rospy.loginfo("There is no second trash bin")
            next_state = "ANNOUNCE_END"

        # drop_zone_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.drop_zone_id)
        helper_waypoint_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.helper_waypoint)
        end_waypoint_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.end_waypoint)
        arm_designator = self.empty_arm_designator = ds.UnoccupiedArmDesignator(
            robot,
            {"required_goals": ["handover", "reset", "handover_to_human"], "force_sensor_required": True,
             "required_gripper_types": [arms.GripperTypes.GRASPING]},
            name="empty_arm_designator")

        with self:
            smach.StateMachine.add("START_CHALLENGE_ROBUST",
                                   states.StartChallengeRobust(robot, CHALLENGE_KNOWLEDGE.starting_point),
                                   transitions={"Done": "SAY_START_CHALLENGE",
                                                "Aborted": "SAY_START_CHALLENGE",
                                                "Failed": "SAY_START_CHALLENGE"})

            smach.StateMachine.add("SAY_START_CHALLENGE",
                                   states.Say(robot, "I will start cleaning up the trash", block= True),
                                   transitions={'spoken': "PICK_UP_TRASH"})

            smach.StateMachine.add("PICK_UP_TRASH", PickUpTrash(robot=robot, trashbin_designator=trashbin_designator,
                                                                arm_designator=arm_designator),
                                   transitions={"succeeded": "DROP_DOWN_TRASH",
                                                "failed": "HELPER_WAYPOINT",
                                                "aborted": "ANNOUNCE_END"})

            smach.StateMachine.add("DROP_DOWN_TRASH",
                                   DropDownTrash(robot=robot, drop_zone_id=CHALLENGE_KNOWLEDGE.drop_zone_id),
                                   transitions={"succeeded": "ANNOUNCE_TASK",
                                                "failed": "failed",
                                                "aborted": "aborted"})

            smach.StateMachine.add("ANNOUNCE_TASK",
                                   states.Say(robot, "First bag has been dropped at the collection zone",
                                              block=False),
                                   transitions={'spoken': next_state})

            if next_state == "HELPER_WAYPOINT":

                smach.StateMachine.add("HELPER_WAYPOINT",
                                       states.NavigateToWaypoint(robot=robot,
                                                                 waypoint_designator=helper_waypoint_designator),
                                       transitions={"arrived": "PICK_UP_TRASH2",
                                                    "goal_not_defined": "PICK_UP_TRASH2",
                                                    "unreachable": "PICK_UP_TRASH2"})

                smach.StateMachine.add("PICK_UP_TRASH2", PickUpTrash(robot=robot,
                                                                     trashbin_designator=trashbin_designator2,
                                                                     arm_designator=arm_designator),
                                       transitions={"succeeded": "DROP_DOWN_TRASH2",
                                                    "failed": "ANNOUNCE_END",
                                                    "aborted": "ANNOUNCE_END"})

                smach.StateMachine.add("DROP_DOWN_TRASH2",
                                       DropDownTrash(robot=robot, drop_zone_id=CHALLENGE_KNOWLEDGE.drop_zone_id),
                                       transitions={"succeeded": "ANNOUNCE_TASK2",
                                                    "failed": "failed",
                                                    "aborted": "aborted"})

                smach.StateMachine.add("ANNOUNCE_TASK2",
                                       states.Say(robot, "Second bag has been dropped at the collection zone."
                                                         "All the thrash has been taken care of",
                                                  block=False),
                                       transitions={'spoken': 'ANNOUNCE_END'})

            smach.StateMachine.add("ANNOUNCE_END",
                                   states.Say(robot, "I have finished taking out the trash.",
                                              block=False),
                                   transitions={'spoken': 'NAVIGATE_OUT'})

            smach.StateMachine.add("NAVIGATE_OUT",
                                   states.NavigateToWaypoint(robot=robot,
                                                             waypoint_designator=end_waypoint_designator),
                                   transitions={"arrived": "succeeded",
                                                "goal_not_defined": "succeeded",
                                                "unreachable": "succeeded"})


class TestDummy(smach.StateMachine):
    def __init__(self, dummy_robot):

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", ])

        # Create designators
        dummy_trashbin_designator = ds.EdEntityDesignator(dummy_robot,
                                                    id=CHALLENGE_KNOWLEDGE.trashbin_id2,
                                                    name='trashbin_designator')
        dummy_arm_designator_un = ds.UnoccupiedArmDesignator(
            dummy_robot,
            {"required_goals": ["handover", "reset", "handover_to_human"], "force_sensor_required": True,
             "required_gripper_types": [arms.GripperTypes.GRASPING]})
        dummy_arm_designator_oc = ds.OccupiedArmDesignator(dummy_robot, {"required_goals": ["handover", "reset"],
                                                                         "required_gripper_types": [
                                                                             arms.GripperTypes.GRASPING]})

        with self:
            smach.StateMachine.add("PICK_UP_TRASH", PickUpTrash(dummy_robot, dummy_trashbin_designator,
                                                                dummy_arm_designator_un),
                                   transitions={"succeeded": "TURN_BASE",
                                                "failed": "TURN_BASE",
                                                "aborted": "failed"})

            smach.StateMachine.add("TURN_BASE", states.ForceDrive(dummy_robot, 0, 0, 0.5, 3),
                                   transitions={"done": "DROP_TRASH"})

            smach.StateMachine.add("DROP_TRASH",
                                   DropTrash(dummy_robot, dummy_arm_designator_oc),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})

if __name__ == '__main__':
    import os
    import robot_smach_states.util.designators as ds
    from robot_skills import Hero

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()

    TestDummy(hero).execute()



