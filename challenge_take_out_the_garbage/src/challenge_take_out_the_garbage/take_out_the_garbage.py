# ROS
import smach

# Robot smach states
import robot_smach_states as states
from robocup_knowledge import load_knowledge
import robot_smach_states.util.designators as ds
from challenge_take_out_the_garbage.take_out import DefaultGrabDesignator, GrabSingleItem, TakeOut
CHALLENGE_KNOWLEDGE = load_knowledge('challenge_take_out_the_garbage')



class TakeOutGarbage(smach.StateMachine):
    """ This is my example state machine

    """
    def __init__(self, robot):
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # Create designators
        trashbin_designator = ds.EdEntityDesignator(robot=robot,
                                                    id=CHALLENGE_KNOWLEDGE.trashbin_id)
        trash_designator = DefaultGrabDesignator(robot=robot,
                                                 surface_designator=trashbin_designator,
                                                 area_description="on_top_of")
        trashbin_designator2 = ds.EdEntityDesignator(robot=robot,
                                                     id=CHALLENGE_KNOWLEDGE.trashbin_id2)
        trash_designator2 = DefaultGrabDesignator(robot=robot,
                                                  surface_designator=trashbin_designator2,
                                                  area_description="on_top_of")
        drop_zone_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.drop_zone_id)

        arm_designator = self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot, {}, name="empty_arm_designator")

        with self:

            # Start challenge via StartChallengeRobust
            smach.StateMachine.add("START_CHALLENGE_ROBUST",
                                   states.StartChallengeRobust(robot, CHALLENGE_KNOWLEDGE.starting_point),
                                   transitions={"Done": "TAKE_OUT",
                                                "Aborted": "aborted",
                                                "Failed": "failed"})

            smach.StateMachine.add("TAKE_OUT",
                                   TakeOut(robot=robot, trashbin_designator=trashbin_designator,
                                           trash_designator=trash_designator, drop_designator=drop_zone_designator,
                                           arm_designator=arm_designator),
                                   transitions={"succeeded": "ANNOUNCE_TASK",
                                                "aborted": "aborted",
                                                "failed": "ANNOUNCE_END"})

            if trashbin_designator2 is not None:
                next_state = "TAKE_OUT2"
            else:
                next_state = "ANNOUNCE_END"

            smach.StateMachine.add("ANNOUNCE_TASK",
                                   states.Say(robot, "First bag has been dropped at the collection zone",
                                              block=False),
                                   transitions={'spoken': next_state})

            smach.StateMachine.add("TAKE_OUT2",
                                   TakeOut(robot=robot, trashbin_designator=trashbin_designator2,
                                           trash_designator=trash_designator2, drop_designator=drop_zone_designator,
                                           arm_designator=arm_designator),
                                   transitions={"succeeded": "ANNOUNCE_TASK2",
                                                "aborted": "aborted",
                                                "failed": "ANNOUNCE_END"})

            smach.StateMachine.add("ANNOUNCE_TASK2",
                                   states.Say(robot, "Second bag has been dropped at the collection zone."
                                                     "All the thrash has been taken care of",
                                              block=False),
                                   transitions={'spoken': 'succeeded'})

            smach.StateMachine.add("ANNOUNCE_END",
                                   states.Say(robot, "I have finished taking out the trash.",
                                              block=False),
                                   transitions={'spoken': 'succeeded'})
