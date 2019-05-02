# ROS
import smach

# Robot smach states
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robocup_knowledge import knowledge_loader
from robot_skills.robot import Robot

# Serving drinks
from .get_order import GetOrder

# Knowledge
CHALLENGE_KNOWLEDGE = knowledge_loader.load_knowledge("challenge_serving_drinks")


class ServeOneDrink(smach.StateMachine):
    """ Serves on drink to an operator

    """
    def __init__(self, robot, idx):
        # type: (Robot, int) -> str
        """ Initialization method

        :param robot: robot api object
        :param idx: index of the drink that is served (used for the operator name)
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        drink_str_designator = ds.VariableDesignator(resolve_type=str)
        drink_designator = ds.EdEntityDesignator(robot=robot, type=drink_str_designator)
        bar_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.bar_id)
        arm_designator = ds.UnoccupiedArmDesignator(robot=robot, arm_properties={})
        operator_name = "operator_{}".format(idx)

        with self:

            smach.StateMachine.add(
                "GET_ORDER",
                GetOrder(
                    robot=robot,
                    operator_name=operator_name,
                    drink_designator=drink_str_designator),
                transitions={"succeeded": "INSPECT_BAR",
                             "failed": "failed"}
            )

            # Inspect bar
            # ToDo: optimize so that we don't re-inspect the bar if we already know where the object is located
            smach.StateMachine.add(
                "INSPECT_BAR",
                states.Inspect(
                    robot=robot,
                    entityDes=bar_designator,
                    navigation_area="in_front_of"),
                transitions={"done": "GRASP_DRINK",
                             "failed": "failed"}  # ToDo: fallback?
            )

            # Grasp drink
            smach.StateMachine.add(
                "GRASP_DRINK",
                states.Grab(
                    robot=robot,
                    item=drink_designator,
                    arm=arm_designator),
                transitions={"done": "FIND_OPERATOR",
                             "failed": "failed"}  # ToDo: fallback?
            )

            # Find operator
            smach.StateMachine.add(
                "FIND_OPERATOR",
                states.FindPersonInRoom(
                    robot=robot,
                    area=CHALLENGE_KNOWLEDGE.room_id,
                    name=operator_name,
                    discard_other_labels=True,
                    found_entity_designator=operator_des.writeable),
                transitions={"found": "GOTO_OPERATOR",
                             "not_found": "SAY_NOT_FOUND"}
            )

            # Move to this person
            smach.StateMachine.add(
                'GOTO_OPERATOR',
                states.NavigateToObserve(
                    robot=robot,
                    entity_designator=operator_des),
                transitions={'arrived': 'HAND_OVER',
                             'unreachable': 'SAY_NOT_FOUND',
                             'goal_not_defined': 'abort'})

            # Say not found
            smach.StateMachine.add(
                "SAY_NOT_FOUND",
                states.Say(
                    robot=robot,
                    sentence="I am terribly sorry but I did not find you. "
                             "Please come to me so I can hand over the drink",
                    look_at_standing_person=True),
                transitions={"spoken": "HAND_OVER"}
            )

            # Hand over the drink
            smach.StateMachine.add(
                "HAND_OVER",
                states.HandoverToHuman(
                    robot=robot,
                    arm_designator=arm_designator),
                transitions={"succeeded": "succeeded",
                             "failed": "failed"}  # ToDo: fallback
            )
