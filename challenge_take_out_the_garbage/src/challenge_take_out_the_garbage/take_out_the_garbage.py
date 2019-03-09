# ROS
import smach

# Robot smach states
import robot_smach_states as states
from robocup_knowledge import load_knowledge
import robot_smach_states.util.designators as ds
from challenge_take_out_the_garbage.take_out import DefaultGrabDesignator, GrabSingleItem
challenge_knowledge = load_knowledge('challenge_take_out_the_garbage')

STARTING_POINT = challenge_knowledge.starting_point


class TakeOutGarbage(smach.StateMachine):
    """ This is my example state machine

    """
    def __init__(self, robot):
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # Create designators
        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, robot.leftArm, name="empty_arm_designator")

        trashbin_id = "trashbin"
        trashbin_designator = ds.EdEntityDesignator(robot=robot,
                                                    id=trashbin_id)
        trash_designator = DefaultGrabDesignator(robot=robot,
                                                 surface_designator=trashbin_designator,
                                                 area_description="on_top_of",
                                                 debug=True)

        # Designator dropping area
        # ToDo Make a designated drop area for the trash
        bed_id = "cabinet"
        bed_designator = ds.EdEntityDesignator(robot=robot,
                                                    id=bed_id)

        with self:

            # Start challenge via StartChallengeRobust
            smach.StateMachine.add("START_CHALLENGE_ROBUST",
                                   states.StartChallengeRobust(robot, STARTING_POINT),
                                   transitions={"Done": "TAKE_OUT",
                                                "Aborted": "aborted",
                                                "Failed": "failed"})
            take_out = smach.StateMachine(outcomes=["succeeded", "failed", "aborted"])


            with take_out:
                # Take Out 1
                smach.StateMachine.add("GO_TO_BIN",
                                       states.NavigateToObserve(robot, trashbin_designator),
                                       transitions={"arrived": "INSPECT",
                                                    "goal_not_defined": "aborted",
                                                    "unreachable": "failed"})

                smach.StateMachine.add("INSPECT",
                                       states.Inspect(robot, trashbin_designator),
                                       transitions={"done": "GRAB_TRASH",
                                                    "failed": "failed"})

                smach.StateMachine.add("GRAB_TRASH", GrabSingleItem(robot=robot, grab_designator=trash_designator),
                                       transitions={"succeeded": "GO_TO_COLLECTION_ZONE",
                                                    "failed": "failed"})

                smach.StateMachine.add("GO_TO_COLLECTION_ZONE",
                                       states.NavigateToObserve(robot, bed_designator),
                                       transitions={"arrived": "succeeded",
                                                    "goal_not_defined": "aborted",
                                                    "unreachable": "failed"})
                # #wip
                # smach.StateMachine.add("DROP_TRASH",
                #                        states.Place(robot=robot, item_to_place=trash_designator,
                #                                     place_pose=bed_designator, arm=self.empty_arm_designator,
                #                                     place_volume="on_top_of"),
                #                        transitions={"done": "succeeded",
                #                                     "failed": "failed"})

            smach.StateMachine.add("TAKE_OUT",
                                   take_out,
                                   transitions={"succeeded": "succeeded",
                                                "aborted": "aborted",
                                                "failed": "failed"})


            # # Take Out 1
            # smach.StateMachine.add("GO_TO_BIN",
            #                        states.NavigateToObserve(robot, entity_designator),
            #                        transitions={"Done": "GRAB_TRASH"})
            # smach.StateMachine.add("GRAB_TRASH",
            #                        States.Grab(robot, trash),
            #                        transitions={"Done": "GO_TO_COLLECTION_ZONE"})
            # smach.StateMachine.add("GO_TO_COLLECTION_ZONE",
            #                        States.NavigateToObserve(robot, entity_designator),
            #                        transitions={"Done": "DROP_TRASH"})
            # smach.StateMachine.add("DROP_TRASH",
            #                        State)
            #
            # Take Out 2
