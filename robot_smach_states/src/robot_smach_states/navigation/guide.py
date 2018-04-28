# ROS
import smach

# Smach states
from robot_smach_states.human_interaction.human_interaction import Say
from robot_smach_states.util.designators.ed_designators import EdEntityDesignator

# Navigation
from navigate_to_observe import NavigateToObserve
from navigate_to_symbolic import NavigateToRoom, NavigateToSymbolic
from navigate_to_waypoint import NavigateToWaypoint


def create_navigation_state(robot, target_location):
    """ Creates the correct Navigation state for the target location designator

    :param robot: (Robot) Robot API object
    :param target_location: EdEntityDesignator designating
    :return: NavigateTo state
    """
    # To handle strings, we need to create an EdEntityDesignator out of it
    if isinstance(target_location, str):
        target_location_designator = EdEntityDesignator(robot=robot, id=target_location)
    elif isinstance(target_location, EdEntityDesignator):
        target_location_designator = target_location
    elif isinstance(target_location, dict):
        target_location_designator = None

    # See if we can resolve the EdEntityDesignator
    entity = None
    if target_location_designator is not None:
        entity = target_location_designator.resolve()

    # Now: see what we need to do
    if entity is not None and entity.is_a("waypoint"):  # We know we have a waypoint
        nav_state = NavigateToWaypoint(robot=robot, waypoint_designator=target_location_designator)
    elif entity is not None and entity.is_a("room"):  # We have a room
        nav_state = NavigateToRoom(robot=robot, entity_designator_room=target_location_designator)
    elif isinstance(target_location, dict):  # Assume we can NavigateToSymbolic
        nav_state = NavigateToSymbolic(robot=robot, entity_designator_area_name_map=target_location,
                                       look_at_designator=target_location[0])
    else:
        nav_state = NavigateToObserve(robot=robot, entity_designator=target_location_designator)

    # Return
    return nav_state


class Guide(smach.StateMachine):
    """ Smach state to guide an operator to a designated location
    """
    def __init__(self, robot, target_location, follower=None):
        """ Guides an operator to the designated location

        :param robot: (Robot) Robot API object
        :param target_location: Either:
         * (EdEntityDesignator) designates the location where to guide the follower to
         * (str) to use in an EdEntityDesignator
         * {EdEntityDesignator: str} mapping EdEntityDesignators to a string or designator resolving to a string,
           representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param follower: (EdEntityDesignator) (optional) designates the person (or robot) to guide
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "preempted"])

        nav_machine = create_navigation_state(robot, target_location)

        # ToDo: update
        loc_sentence = "Follow me if you want to make it out of here alive"
        arrived_sentence = "Yeehah, we have arrived"
        failed_sentence = "I lost my way, I'm so sorry"
        no_goal_sentence = "I'm not sure where I'm supposed to go"

        # End of ToDo

        with self:
            smach.StateMachine.add("ANNOUNCE_DEPARTURE", Say(robot, loc_sentence, mood="angry",
                                                             block=True, look_at_standing_person=True),
                                   transitions={"spoken": "NAVIGATE"})

            smach.StateMachine.add("NAVIGATE", nav_machine,
                                   transitions={"arrived": "ANNOUNCE_ARRIVAL",
                                                "unreachable": "ANNOUNCE_FAILURE",
                                                "goal_not_defined": "ANNOUNCE_NO_GOAL"})

            smach.StateMachine.add("ANNOUNCE_ARRIVAL",
                                   Say(robot, arrived_sentence, mood="excited", block=True,
                                       look_at_standing_person=True),
                                   transitions={"spoken": "succeeded"})

            smach.StateMachine.add("ANNOUNCE_FAILURE",
                                   Say(robot, failed_sentence, mood="sad", block=True, look_at_standing_person=True),
                                   transitions={"spoken": "failed"})

            smach.StateMachine.add("ANNOUNCE_NO_GOAL",
                                   Say(robot, no_goal_sentence, mood="furious", block=True,
                                       look_at_standing_person=True),
                                   transitions={"spoken": "succeeded"})
