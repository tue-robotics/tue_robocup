# ROS
import smach

# Robot smach states
import robot_smach_states as states
from robocup_knowledge import load_knowledge
import robot_smach_states.util.designators as ds
from challenge_take_out_the_garbage.pick_up import PickUpTrash
from challenge_take_out_the_garbage.drop_down import DropDownTrash
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
                                                    id=CHALLENGE_KNOWLEDGE.trashbin_id)
        trash_designator = ds.EntityByIdDesignator(robot=robot, id="trash")

        # Look if there is a second trash bin present
        if CHALLENGE_KNOWLEDGE.trashbin_id2 is not None:
            trashbin_designator2 = ds.EdEntityDesignator(robot=robot,
                                                         id=CHALLENGE_KNOWLEDGE.trashbin_id2)
            trash_designator2 = ds.EntityByIdDesignator(robot=robot, id="trash")

        drop_zone_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.drop_zone_id)
        arm_designator = self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot, {}, name="empty_arm_designator")

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
                                                "failed": "ANNOUNCE_END",
                                                "aborted": "ANNOUNCE_END"})

            smach.StateMachine.add("DROP_DOWN_TRASH",
                                   DropDownTrash(robot=robot, trash_designator=trash_designator,
                                                 drop_designator=drop_zone_designator),
                                   transitions={"succeeded": "ANNOUNCE_TASK",
                                                "failed": "failed",
                                                "aborted": "aborted"})
            if trash_designator2 is not None:
                next_state = "PICK_UP_TRASH2"
            else:
                next_state = "ANNOUNCE_END"

            smach.StateMachine.add("ANNOUNCE_TASK",
                                   states.Say(robot, "First bag has been dropped at the collection zone",
                                              block=False),
                                   transitions={'spoken': next_state})

            smach.StateMachine.add("PICK_UP_TRASH2", PickUpTrash(robot=robot, trashbin_designator=trashbin_designator2,
                                                                 arm_designator=arm_designator),
                                   transitions={"succeeded": "DROP_DOWN_TRASH2",
                                                "failed": "ANNOUNCE_END",
                                                "aborted": "ANNOUNCE_END"})

            smach.StateMachine.add("DROP_DOWN_TRASH2",
                                   DropDownTrash(robot=robot, trash_designator=trash_designator2,
                                                 drop_designator=drop_zone_designator),
                                   transitions={"succeeded": "ANNOUNCE_TASK2",
                                                "failed": "failed",
                                                "aborted": "aborted"})

            smach.StateMachine.add("ANNOUNCE_TASK2",
                                   states.Say(robot, "Second bag has been dropped at the collection zone."
                                                     "All the thrash has been taken care of",
                                              block=False),
                                   transitions={'spoken': 'succeeded'})

            smach.StateMachine.add("ANNOUNCE_END",
                                   states.Say(robot, "I have finished taking out the trash.",
                                              block=False),
                                   transitions={'spoken': 'succeeded'})

