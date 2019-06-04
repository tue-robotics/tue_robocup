# ROS
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds

from robocup_knowledge import knowledge_loader
from robot_skills.robot import Robot

# Serving drinks
from .sd_states import AskDrink, DetectWaving

# Knowledge
COMMON_KNOWLEDGE = knowledge_loader.load_knowledge("common")


class GetOrder(smach.StateMachine):
    """ Gets an order. If succeeded, the person_designator and drink_designator are filled and can be used in subsequent
    states.

    """
    def __init__(self, robot, operator_name, drink_designator,
                 available_drinks_designator, unavailable_drink_designator):
        # type: (Robot, str, VariableDesignator) -> None
        """ Initialization method

        :param robot: robot api object
        :param operator_name: name with which the operator will be stored in image recognition module
        :param drink_designator: (VariableDesignator) in which the drink to fetch is stored.
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:

            # Operator id
            caller_id = "operator"
            caller_designator = ds.EdEntityDesignator(robot=robot, id=caller_id, name='caller_des')

            # Detect - people holding drinks and people without drinks  #ToDo: implement!

            # Detect fallback - detect waving people
            smach.StateMachine.add(
                "ASK_FOR_WAVING",
                states.Say(
                    robot=robot,
                    sentence="Looks like everyone has a drink",
                    look_at_standing_person=True),
                transitions={"spoken": "ASK_STEP_IN_FRONT"}  # ToDo: transition to WAIT_FOR_WAVING
            )

            smach.StateMachine.add(
                "WAIT_FOR_WAVING",
                DetectWaving(
                    robot=robot,
                    caller_id=caller_id),
                transitions={'succeeded': 'SAY_I_HAVE_SEEN',
                             'aborted': 'ASK_STEP_IN_FRONT'}
            )

            smach.StateMachine.add(
                'SAY_I_HAVE_SEEN',
                states.Say(
                    robot=robot,
                    sentence='I have seen a waving person, I will be there shortly!',
                    look_at_standing_person=True),
                transitions={"spoken": 'NAVIGATE_TO_WAVING'}
            )

            # Navigate to waving people
            smach.StateMachine.add(
                'NAVIGATE_TO_WAVING',
                states.NavigateToObserve(
                    robot=robot,
                    entity_designator=caller_designator,
                    radius=1.1),
                transitions={'arrived': 'LEARN_NAME',
                             'unreachable': 'ASK_STEP_IN_FRONT',
                             'goal_not_defined': 'WAIT_FOR_WAVING'}
            )

            # Detect waving people fallback - ask operator in front
            smach.StateMachine.add(
                "ASK_STEP_IN_FRONT",
                states.Say(
                    robot=robot,
                    sentence="Please step in front of me to give your order",
                    look_at_standing_person=True),
                transitions={"spoken": "LEARN_NAME"}
            )

            # Ask operator for his name
            smach.StateMachine.add(
                "LEARN_NAME",
                states.AskPersonName(
                    robot=robot,
                    person_name_des=operator_name.writeable,
                    name_options=COMMON_KNOWLEDGE.names,
                    default_name='john',
                    nr_tries=2),
                transitions={"succeeded": "LEARN_OPERATOR",
                             "failed": "LEARN_NAME_FALLBACK",
                             "timeout": "LEARN_NAME_FALLBACK"}
            )

            # Ask operator for his name fallback
            smach.StateMachine.add(
                "LEARN_NAME_FALLBACK",
                states.Say(
                    robot=robot,
                    sentence="Sorry, I did not get your name, I'll just call you john",
                    look_at_standing_person=True),
                transitions={"spoken": "LEARN_OPERATOR"}
            )

            # Learn operator
            smach.StateMachine.add(
                "LEARN_OPERATOR",
                states.LearnPerson(
                    robot=robot,
                    name_designator=operator_name,
                    nr_tries=5),
                transitions={"succeeded": "ASK_DRINK",
                             "failed": "LEARN_OPERATOR_FALLBACK"}
            )

            # Learn operator fallback
            smach.StateMachine.add(
                "LEARN_OPERATOR_FALLBACK",
                states.Say(
                    robot=robot,
                    sentence="Something went wrong but I will call you by name when I'm back",
                    look_at_standing_person=True),
                transitions={"spoken": "ASK_DRINK"}
            )

            # Ask for preferred beverage
            smach.StateMachine.add(
                "ASK_DRINK",
                AskDrink(
                    robot=robot,
                    operator_name=operator_name,
                    drink_designator=drink_designator.writeable,
                    available_drinks_designator=available_drinks_designator,
                    unavailable_drink_designator=unavailable_drink_designator),
                transitions={"succeeded": "succeeded",
                             "failed": "failed"},
            )
