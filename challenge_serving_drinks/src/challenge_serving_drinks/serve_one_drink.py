# ROS
import smach

# Robot smach states
import robot_smach_states as states
import robot_smach_states.util.designators as ds

from robot_skills.util.entity import Entity

# Serving drinks
from .sd_states import DescriptionStrDesignator
from .get_order import GetOrder


class ServeOneDrink(smach.StateMachine):
    """
    Serves on drink to an operator
    """
    def __init__(self, robot, bar_designator, room_id, room_designator,
                 objects_list_des, unav_drink_des, name_options, objects):
        """
        Initialization method

        :param robot: robot api object
        :param bar_designator: (EntityDesignator) in which the bar location is stored
        :param room_id: room ID from challenge knowledge
        :param room_designator: (EntityDesignator) in which the room location is stored
        :param objects_list_des: (VariableDesignator) in which the available drinks are stored
        :param unav_drink_des: (VariableDesignator) in which the unavailable drink is stored
        :param name_options: Names from common knowledge
        :param objects: Objects from common knowledge
        """

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # Designators
        arm_designator = ds.UnoccupiedArmDesignator(robot=robot,
                                                    arm_properties={},
                                                    name='arm_des').lockable()

        drink_str_designator = ds.VariableDesignator(resolve_type=str, name='drink_str_des')
        drink_designator = ds.EdEntityDesignator(robot=robot, type_designator=drink_str_designator, name='drink_des')

        operator_name = ds.VariableDesignator(resolve_type=str, name='name_des')
        operator_designator = ds.VariableDesignator(resolve_type=Entity, name='operator_des')
        learn_check_designator = ds.VariableDesignator(initial_value=True, resolve_type=bool, name='learn_check_des')
        hacky_arm_des = ds.VariableDesignator(initial_value=robot.get_arm(), name='hacky_arm_2')

        with self:

            # Lock the arm_designator
            smach.StateMachine.add("LOCK_ARM",
                                   states.LockDesignator(arm_designator),
                                   transitions={'locked': "GET_ORDER"})

            # Get order
            smach.StateMachine.add("GET_ORDER",
                                   GetOrder(robot=robot,
                                            operator_name=operator_name,
                                            drink_designator=drink_str_designator,
                                            available_drinks_designator=objects_list_des,
                                            unavailable_drink_designator=unav_drink_des,
                                            name_options=name_options,
                                            objects=objects,
                                            learn_check_designator=learn_check_designator.writeable,
                                            target_room_designator=room_designator),
                                   transitions={"succeeded": "INSPECT_BAR",
                                                "failed": "failed",
                                                "aborted": "aborted"})

            # Inspect bar
            smach.StateMachine.add("INSPECT_BAR",
                                   states.Inspect(robot=robot, entityDes=bar_designator, navigation_area="in_front_of"),
                                   transitions={"done": "GRASP_DRINK",
                                                "failed": "FALLBACK_BAR"})

            # Grasp drink
            smach.StateMachine.add("GRASP_DRINK",
                                   states.Grab(robot=robot, item=drink_designator, arm=arm_designator),
                                   transitions={"done": "FIND_OPERATOR",
                                                "failed": "FALLBACK_BAR"})

            # Inspect or grasp fallback - ask for assistance
            smach.StateMachine.add("FALLBACK_BAR",
                                   states.Say(robot=robot,
                                              sentence=DescriptionStrDesignator("fallback_bar", drink_str_designator,
                                                                                operator_name),
                                              look_at_standing_person=True),
                                   transitions={"spoken": "HANDOVER_FROM_HUMAN"})

            # Handover from human fallback
            smach.StateMachine.add("HANDOVER_FROM_HUMAN",
                                   states.HandoverFromHuman(robot=robot, arm_designator=arm_designator,
                                                            grabbed_entity_designator=drink_designator),
                                   transitions={"succeeded": "RESET_ROBOT_2",
                                                "failed": "RESET_ROBOT_2",
                                                "timeout": "RESET_ROBOT_2"})

            smach.StateMachine.add("RESET_ROBOT_2",
                                   states.ArmToJointConfig(robot=robot,
                                                           arm_designator=hacky_arm_des,
                                                           configuration="reset"),
                                   transitions={'succeeded': "CHECK_LEARN_OPERATOR",
                                                'failed': "CHECK_LEARN_OPERATOR"})

            smach.StateMachine.add("CHECK_LEARN_OPERATOR",
                                   states.CheckBool(learn_check_designator),
                                   transitions={"true": "FIND_OPERATOR",
                                                "false": "GO_TO_ROOM"})

            smach.StateMachine.add("GO_TO_ROOM",
                                   states.NavigateToRoom(robot=robot,
                                                         entity_designator_room=room_designator),
                                   transitions={"arrived": "SAY_NOT_FOUND",
                                                "unreachable": "failed",
                                                "goal_not_defined": "aborted"})
            # Find operator
            smach.StateMachine.add("FIND_OPERATOR",
                                   states.FindPersonInRoom(robot=robot,
                                                           area=room_id,
                                                           name=operator_name,
                                                           discard_other_labels=True,
                                                           found_entity_designator=operator_designator.writeable),
                                   transitions={"found": "GOTO_OPERATOR",
                                                "not_found": "SAY_NOT_FOUND"})

            # Move to this person
            smach.StateMachine.add("GOTO_OPERATOR",
                                   states.NavigateToObserve(robot=robot,
                                                            entity_designator=operator_designator),
                                   transitions={"arrived": "SAY_THE_NAME",
                                                "unreachable": "SAY_NOT_FOUND",
                                                "goal_not_defined": "SAY_NOT_FOUND"})

            # Say not found
            smach.StateMachine.add("SAY_NOT_FOUND",
                                   states.Say(robot=robot,
                                              sentence=DescriptionStrDesignator("not_found_operator",
                                                                                drink_str_designator,
                                                                                operator_name),
                                              look_at_standing_person=True),
                                   transitions={"spoken": "RISE_FOR_HMI_2"})

            # Say the name
            smach.StateMachine.add("SAY_THE_NAME",
                                   states.Say(robot=robot,
                                              sentence=DescriptionStrDesignator("found_operator",
                                                                                drink_str_designator,
                                                                                operator_name),
                                              look_at_standing_person=True),
                                   transitions={"spoken": "RISE_FOR_HMI_2"})

            smach.StateMachine.add("RISE_FOR_HMI_2",
                                   states.RiseForHMI(robot=robot),
                                   transitions={"succeeded": "HAND_OVER",
                                                "failed": "HAND_OVER"})

            # Hand over the drink to the operator
            smach.StateMachine.add("HAND_OVER",
                                   states.HandoverToHuman(robot=robot,
                                                          arm_designator=arm_designator),
                                   transitions={"succeeded": "UNLOCK_ARM",
                                                "failed": "UNLOCK_ARM"})

            smach.StateMachine.add("UNLOCK_ARM",
                                   states.UnlockDesignator(arm_designator),
                                   transitions={'unlocked': "RESET_ROBOT_3"})

            smach.StateMachine.add("RESET_ROBOT_3",
                                   states.ArmToJointConfig(robot=robot,
                                                           arm_designator=hacky_arm_des,
                                                           configuration="reset"),
                                   transitions={'succeeded': "RETURN_TO_ROOM",
                                                'failed': "RETURN_TO_ROOM"})

            smach.StateMachine.add("RETURN_TO_ROOM",
                                   states.NavigateToRoom(robot=robot,
                                                         entity_designator_room=room_designator),
                                   transitions={"arrived": "succeeded",
                                                "unreachable": "failed",
                                                "goal_not_defined": "aborted"})
