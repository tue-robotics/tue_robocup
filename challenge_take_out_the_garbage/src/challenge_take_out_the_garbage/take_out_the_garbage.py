# ROS
import smach

# Robot smach states
import robot_smach_states as states
from robocup_knowledge import load_knowledge

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

        trashbin_id = "trashbin"
        trashbin_designator = states.util.designators.ed_designators.EdEntityDesignator(robot=robot,
                                                                                        id=trashbin_id)


        with self:

            # Start challenge via StartChallengeRobust
            smach.StateMachine.add("START_CHALLENGE_ROBUST",
                                   states.StartChallengeRobust(robot, STARTING_POINT, use_entry_points=True),
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
                                       transitions={"done": "succeeded",
                                                    "failed": "failed"})

                # smach.StateMachine.add("GRAB_TRASH",
                #                        States.Grab(robot, trash),
                #                        transitions={"Done": "GO_TO_COLLECTION_ZONE"})

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
