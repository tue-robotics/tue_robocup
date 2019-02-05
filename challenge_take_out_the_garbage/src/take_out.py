class TakeOut(smach.StateMachine):
    """ This is my example state machine

    """
    def __init__(self, robot):
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:

            # # Take Out 1
            smach.StateMachine.add("GO_TO_BIN",
                                  states.NavigateToObserve(robot, entity_designator),
                                  transitions={"Done": "GRAB_TRASH"})
            # smach.StateMachine.add("GRAB_TRASH",
            #                        States.Grab(robot, trash),
            #                        transitions={"Done": "GO_TO_COLLECTION_ZONE"})
            # smach.StateMachine.add("GO_TO_COLLECTION_ZONE",
            #                        States.NavigateToObserve(robot, entity_designator),
            #                        transitions={"Done": "DROP_TRASH"})
            # smach.StateMachine.add("DROP_TRASH",
            #                        State)
