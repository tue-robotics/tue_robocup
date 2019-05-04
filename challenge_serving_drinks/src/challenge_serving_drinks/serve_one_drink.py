# ROS
import smach
import rospy

# Robot smach states
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util.entity import Entity
from robocup_knowledge import knowledge_loader
from robot_skills.robot import Robot

# Serving drinks
from .sd_states import DescriptionStrDesignator
from .get_order import GetOrder

# Knowledge
CHALLENGE_KNOWLEDGE = knowledge_loader.load_knowledge("challenge_serving_drinks")


class ServeOneDrink(smach.StateMachine):
    """ Serves on drink to an operator

    """
    def __init__(self, robot):
        # type: (Robot, int) -> str
        """ Initialization method

        :param robot: robot api object
        :param idx: index of the drink that is served (used for the operator name)
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        drink_str_designator = ds.VariableDesignator(resolve_type=str, name='drink_str_des')
        drink_designator = ds.EdEntityDesignator(robot=robot, type_designator=drink_str_designator, name='drink_des')

        room_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.room_id, name='room_des')
        bar_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.bar_id, name='bar_des')
        arm_designator = ds.UnoccupiedArmDesignator(robot=robot, arm_properties={}, name='arm_des')

        operator_name = ds.VariableDesignator(resolve_type=str, name='name_des')
        operator_designator = ds.VariableDesignator(resolve_type=Entity, name='operator_des')

        with self:

            # Get order
            smach.StateMachine.add(
                "GET_ORDER",
                GetOrder(
                    robot=robot,
                    operator_name=operator_name,
                    drink_designator=drink_str_designator),
                transitions={"succeeded": "INSPECT_BAR",
                             "failed": "failed"}  # ToDo: fallback?
            )

            # Inspect bar
            # ToDo: optimize so that we don't re-inspect the bar if we already know where the object is located?
            smach.StateMachine.add(
                "INSPECT_BAR",
                states.Inspect(
                    robot=robot,
                    entityDes=bar_designator,
                    navigation_area="in_front_of"),
                transitions={"done": "GRASP_DRINK",
                             "failed": "FALLBACK_BAR"}
            )

            # Grasp drink
            smach.StateMachine.add(
                "GRASP_DRINK",
                states.Grab(
                    robot=robot,
                    item=drink_designator,
                    arm=arm_designator),
                transitions={"done": "FIND_OPERATOR",
                             "failed": "FALLBACK_BAR"}
            )

            # Inspect or grasp fallback - ask for assistance
            smach.StateMachine.add(
                "FALLBACK_BAR",
                states.Say(
                    robot=robot,
                    sentence=DescriptionStrDesignator(operator_name, drink_str_designator, "fallback_bar"),
                    look_at_standing_person=True),
                transitions={"spoken": "HANDOVER_FROM_HUMAN"}
            )

            # Handover from human fallback
            smach.StateMachine.add(
                "HANDOVER_FROM_HUMAN",
                states.HandoverFromHuman(
                    robot=robot,
                    arm_designator=arm_designator,
                    grabbed_entity_designator=drink_designator),
                transitions={"succeeded": "FIND_OPERATOR",
                             "failed": "FIND_OPERATOR",  # ToDo: fallback?
                             "timeout": "FIND_OPERATOR"}  # ToDo: fallback?
            )

            # Find operator
            smach.StateMachine.add(
                "FIND_OPERATOR",
                states.FindPersonInRoom(
                    robot=robot,
                    area=CHALLENGE_KNOWLEDGE.room_id,
                    name=operator_name,
                    discard_other_labels=True,
                    found_entity_designator=operator_designator.writeable),
                transitions={"found": "GOTO_OPERATOR",
                             "not_found": "SAY_NOT_FOUND"}
            )

            # Move to this person
            smach.StateMachine.add(
                'GOTO_OPERATOR',
                states.NavigateToObserve(
                    robot=robot,
                    entity_designator=operator_designator),
                transitions={'arrived': 'SAY_THE_NAME',
                             'unreachable': 'SAY_NOT_FOUND',
                             'goal_not_defined': 'SAY_NOT_FOUND'})

            # Say not found
            smach.StateMachine.add(
                "SAY_NOT_FOUND",
                states.Say(
                    robot=robot,
                    sentence=DescriptionStrDesignator(operator_name, drink_str_designator, "not_found_operator"),
                    look_at_standing_person=True),
                transitions={"spoken": "HAND_OVER"}
            )

            # Say the name
            smach.StateMachine.add(
                "SAY_THE_NAME",
                states.Say(
                    robot=robot,
                    sentence=DescriptionStrDesignator(operator_name, drink_str_designator, "found_operator"),
                    look_at_standing_person=True),
                transitions={"spoken": "HAND_OVER"}
            )

            # Hand over the drink to the operator
            smach.StateMachine.add(
                "HAND_OVER",
                states.HandoverToHuman(
                    robot=robot,
                    arm_designator=arm_designator),
                transitions={"succeeded": "RETURN_TO_ROOM",
                             "failed": "RETURN_TO_ROOM"}
            )

            smach.StateMachine.add(
                "RETURN_TO_ROOM",
                states.NavigateToRoom(
                    robot=robot,
                    entity_designator_room=room_designator),
                transitions={"arrived": "succeeded",
                             "unreachable": "failed",
                             "goal_not_defined": "aborted"}
            )
