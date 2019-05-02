# ROS
import smach

# TU/e Robotics
import robot_smach_states as states
from robot_skills.robot import Robot
from robocup_knowledge import knowledge_loader

# Challenge serving drinks
from .serve_one_drink import ServeOneDrink

# Knowledge
CHALLENGE_KNOWLEDGE = knowledge_loader.load_knowledge("challenge_serving_drinks")

NR_DRINKS = 3


class ServingDrinks(smach.StateMachine):
    """ State machine for 'Serving Drinks' challenge.

    """

    def __init__(self, robot):
        # type: (Robot) -> str
        """ Initialization method

        :param robot: robot api object
        """

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:
            # ToDo: check in rulebook how challenge starts
            # Start challenge via StartChallengeRobust
            # smach.StateMachine.add(
            #     'START_CHALLENGE_ROBUST',
            #     states.StartChallengeRobust(robot, CHALLENGE_KNOWLEDGE.starting_point, use_entry_points=False),
            #     transitions={'Done': 'SAY_HI',
            #                  'Aborted': 'aborted',
            #                  'Failed': 'SAY_HI'}
            # )

            smach.StateMachine.add(
                "SAY_HI",
                states.Say(robot, "Hi, I am {}. I will be your waiter today".format(robot.robot_name)),
                transitions={"spoken": "SERVE_DRINK_1"}
            )

            # Explicitly add a new state for each drink, i.e., don't use a range iterator to make sure a new state
            # is constructed every time
            for idx in range(1, NR_DRINKS + 1):
                next_state = "SERVE_DRINK_{}".format(idx + 1) if idx < NR_DRINKS else "SAY_DONE"

                smach.StateMachine.add(
                    "SERVE_DRINK_{}".format(idx),
                    ServeOneDrink(robot, idx),
                    transitions={"succeeded": next_state,
                                 "failed": next_state,
                                 "aborted": "aborted"}
                )

            smach.StateMachine.add(
                "SAY_DONE",
                states.Say(robot, "My job here is done. Enjoy your day and see you next time"),
                transitions={"spoken": "succeeded"})
