# ROS
import smach

# TU/e Robotics
from robocup_knowledge import knowledge_loader
import robot_smach_states as states

# Challenge final
from track_operator import TrackFace

# Load the knowledge
knowledge = knowledge_loader.load_knowledge("challenge_final")


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
                                   transitions={"succeeded": "TRACK_OPERATOR",
                                                "failed": "Done"})

            # TrackOperator state
            smach.StateMachine.add("TRACK_OPERATOR",
                                   TrackFace(robot=robot),
                                   transitions={"aborted": "Done",
                                                "lost": "Done"})
