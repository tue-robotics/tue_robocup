# ROS
import smach

# TU/e Robotics
import robot_smach_states as states


class ChallengeFinal(smach.StateMachine):
    """ State machine for the final challenge """
    def __init__(self, robot):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        with self:
            # Start challenge
            smach.StateMachine.add("START_CHALLENGE",
                                   states.StartChallengeRobust(robot=robot, initial_pose=knowledge.initial_pose),
                                   transitions={"Done": "LEARN_OPERATOR",
                                                "Aborted": "Aborted",
                                                "Failed": "Aborted"})

            # Learn operator
            smach.StateMachine.add("LEARN_OPERATOR",
                                   states.LearnOperator(robot, person_name="operator", nr_tries=5),
                                   transitions={"succeeded": "Done",
                                                "failed": "Done"})

            # TrackOperator state
