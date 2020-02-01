# ROS
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds

# Serving drinks
from .sd_states import AskDrink

class GetOrder(smach.StateMachine):
    """
    Gets an order. If succeeded, the person_designator and drink_designator are filled and can be used in subsequent
    states.
    """
    def __init__(self, robot, operator_name, drink_designator,
                 available_drinks_designator, unavailable_drink_designator,
                 name_options, objects, learn_check_designator, target_room_designator):
        # type: (Robot, str, VariableDesignator) -> None
        """
        Initialization method

        :param robot: robot api object
        :param operator_name: name with which the operator will be stored in image recognition module
        :param drink_designator: (VariableDesignator) in which the drink to fetch is stored
        :param available_drinks_designator: (VariableDesignator) in which the available drinks are stored
        :param unavailable_drink_designator: (VariableDesignator) in which the unavailable drink is stored
        :param name_options: Names from common knowledge
        :param objects: Objects from common knowledge
        :param learn_check_designator: (VariableDesignator) Bool flag indicating whether the operator was learnt
            successfully
        :param target_room_designator: (EdEntityDesignator) Entity specifying the target room where the operator needs
            to be searched for getting an order
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])
        hacky_arm_des = ds.VariableDesignator(initial_value=robot.get_arm(), name='hacky_arm_3')

        with self:

            # Operator id
            caller_id = "operator"
            caller_designator = ds.EdEntityDesignator(robot=robot, id=caller_id, name="caller_des", none_resolve=True)
            smach.StateMachine.add("RESET_ROBOT_GET_ORDER",
                                   states.ArmToJointConfig(robot=robot,
                                                           arm_designator=hacky_arm_des,
                                                           configuration="reset"),
                                   transitions={'succeeded': "SAY_PEOPLE_WITHOUT_DRINKS",
                                                'failed': "SAY_PEOPLE_WITHOUT_DRINKS"})

            # Detect - people holding drinks and people without drinks
            smach.StateMachine.add("SAY_PEOPLE_WITHOUT_DRINKS",
                                   states.Say(robot=robot, sentence="Trying to find people without a drink",
                                              look_at_standing_person=True,
                                              block=True),
                                   transitions={"spoken": "FIND_PERSON_WITHOUT_DRINK"})

            # TODO: Change DummyState to actual state
            smach.StateMachine.add("FIND_PERSON_WITHOUT_DRINK",
                                   states.SetPoseFirstFoundPersonToEntity(robot=robot,
                                                                          properties={'tags': ['LNotHolding', 'RNotHolding']},
                                                                          strict=True,
                                                                          dst_entity_designator=caller_id,
                                                                          query_entity_designator=target_room_designator),
                                   transitions={"done": "SAY_I_HAVE_SEEN",
                                                "failed": "SAY_PEOPLE_WITHOUT_DRINKS_FAILED"})

            # Detect fallback - detect waving people
            smach.StateMachine.add("SAY_PEOPLE_WITHOUT_DRINKS_FAILED",
                                   states.Say(robot=robot,
                                              sentence="Could not detect people without drinks",
                                              look_at_standing_person=True,
                                              block=True),
                                   transitions={"spoken": "ASK_FOR_WAVING"})

            smach.StateMachine.add("ASK_FOR_WAVING",
                                   states.Say(robot=robot,
                                              sentence="Please raise your arm completely and wave, if you want me to bring you something",
                                              look_at_standing_person=True,
                                              block=True),
                                   transitions={"spoken": "WAIT_FOR_WAVING"}) # Change to WAIT_FOR_WAVING

            smach.StateMachine.add("WAIT_FOR_WAVING",
                                   states.SetPoseFirstFoundPersonToEntity(robot=robot,
                                                                          properties={'tags': ['LWave', 'RWave']},
                                                                          strict=False,
                                                                          dst_entity_designator=caller_id,
                                                                          query_entity_designator=target_room_designator),
                                   transitions={"done": "SAY_I_HAVE_SEEN",
                                                "failed": "SAY_COULD_NOT_FIND_WAVING"})

            # Navigate to person who wants to place an order
            smach.StateMachine.add("SAY_COULD_NOT_FIND_WAVING",
                                   states.Say(robot=robot,
                                              sentence="I did not find any waving person.",
                                              look_at_standing_person=True,
                                              block=True),
                                   transitions={"spoken": "ASK_STEP_IN_FRONT"})

            smach.StateMachine.add("SAY_I_HAVE_SEEN",
                                   states.Say(robot=robot,
                                              sentence="Found person who might want to place an order. I will be there shortly!",
                                              look_at_standing_person=True,
                                              block=True),
                                   transitions={"spoken": "NAVIGATE_TO_PERSON"})

            # Navigate to waving people
            smach.StateMachine.add("NAVIGATE_TO_PERSON",
                                   states.NavigateToObserve(robot=robot, entity_designator=caller_designator,
                                                            radius=1.1),
                                   transitions={"arrived": "LEARN_NAME",
                                                "unreachable": "SAY_COULD_NOT_NAVIGATE",
                                                "goal_not_defined": "SAY_PEOPLE_WITHOUT_DRINKS"})

            # Detect waving people fallback - ask operator in front
            smach.StateMachine.add("SAY_COULD_NOT_NAVIGATE",
                                   states.Say(robot=robot,
                                              sentence="Sorry! I could not navigate to you.",
                                              look_at_standing_person=True),
                                   transitions={"spoken": "ASK_STEP_IN_FRONT"})

            smach.StateMachine.add("ASK_STEP_IN_FRONT",
                                   states.Say(robot=robot,
                                              sentence="Please step in front of me to give your order",
                                              look_at_standing_person=True,
                                              block=True),
                                   transitions={"spoken": "LEARN_NAME"})

            # Ask operator for his name
            smach.StateMachine.add("LEARN_NAME",
                                   states.AskPersonName(robot=robot,
                                                        person_name_des=operator_name.writeable,
                                                        name_options=name_options,
                                                        default_name="john",
                                                        nr_tries=2),
                                   transitions={"succeeded": "LEARN_OPERATOR",
                                                "failed": "LEARN_NAME_FALLBACK",
                                                "timeout": "LEARN_NAME_FALLBACK"})

            # Ask operator for his name fallback
            smach.StateMachine.add("LEARN_NAME_FALLBACK",
                                   states.Say(robot=robot,
                                              sentence="Sorry, I did not get your name, I'll just call you john",
                                              look_at_standing_person=True),
                                   transitions={"spoken": "LEARN_OPERATOR"})

            # Learn operator
            smach.StateMachine.add("LEARN_OPERATOR",
                                   states.LearnPerson(robot=robot,
                                                      name_designator=operator_name,
                                                      nr_tries=5),
                                   transitions={"succeeded": "ASK_DRINK",
                                                "failed": "LEARN_OPERATOR_FALLBACK"})

            # Learn operator fallback
            smach.StateMachine.add("LEARN_OPERATOR_FALLBACK",
                                   states.Say(robot=robot,
                                              sentence="I will call you by your name when I'm back",
                                              look_at_standing_person=True,
                                              block=True),
                                   transitions={"spoken": "LEARN_OPERATOR_FLAG_TOGGLE"})

            smach.StateMachine.add("LEARN_OPERATOR_FLAG_TOGGLE",
                                   states.ToggleBool(learn_check_designator),
                                   transitions={"done": "ASK_DRINK"})

            # Ask for preferred beverage
            smach.StateMachine.add("ASK_DRINK",
                                   AskDrink(robot=robot,
                                            operator_name=operator_name,
                                            drink_designator=drink_designator.writeable,
                                            available_drinks_designator=available_drinks_designator,
                                            unavailable_drink_designator=unavailable_drink_designator,
                                            objects=objects),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed",
                                                "aborted": "aborted"})


if __name__ == "__main__":
    from robot_skills import get_robot
    from robot_skills.classification_result import ClassificationResult
    import rospy
    from robocup_knowledge import knowledge_loader
    import sys
    common_knowledge = knowledge_loader.load_knowledge("common")

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

        rospy.init_node('test_get_order')

        robot = get_robot(robot_name)
        operator_name = ds.VariableDesignator(resolve_type=str, name='name_des')
        drink_str_designator = ds.VariableDesignator(resolve_type=str, name='drink_str_des')

        available_drinks = [ClassificationResult('beer', None, None, None),
                            ClassificationResult('juice', None, None, None),
                            ClassificationResult('coke', None, None, None)]

        available_drinks_designator = ds.VariableDesignator(initial_value=available_drinks,
                                                            resolve_type=[ClassificationResult],
                                                            name='objects_list_des')

        unavailable_drink_designator = ds.VariableDesignator(initial_value="tea_bag",
                                                             resolve_type=str,
                                                             name='unav_drink_str_des')

        name_options = common_knowledge.names
        objects = common_knowledge.objects
        learn_check_designator = ds.VariableDesignator(initial_value=True, resolve_type=bool, name='learn_check_des')

        sm = GetOrder(robot,
                      operator_name,
                      drink_str_designator,
                      available_drinks_designator,
                      unavailable_drink_designator,
                      name_options,
                      objects,
                      learn_check_designator.writeable)
        sm.execute()
    else:
        print("Please provide robot_name as argument. Eg. 'hero'")
        exit(1)

