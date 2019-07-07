# ROS
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds

# Challenge final
from .display_orders_on_map import DisplayOrdersOnMap
from .navigate_to_and_grab_rack import NavigateToAndGrabRack

RACK_ID = "rack"
RACK_NAVIGATION_AREA = "in_front_of"
FRIDGE_WAYPOINT = "final_fridge_waypoint"
WAYPOINT_ID = "find_people_waypoint"


class GetDrinks(smach.StateMachine):
    def __init__(self, robot):
        """
        Grabs the rack, waits for the drinks to be put in the rack and drives with the rack to deliver the drinks.

        :param robot: (Robot) api object
        """
        smach.StateMachine.__init__(self, outcomes=["done", "failed"], input_keys=["detected_people"])

        with self:

            # Say need something to carry drinks
            smach.StateMachine.add("SAY_NEED_RACK",
                                   states.Say(
                                       robot,
                                       "I cannot carry this many drinks. I better get something to make it easier",
                                       block=False),
                                   transitions={"spoken": "GRAB_RACK"}
                                   )

            # Navigate to and grab the rack
            smach.StateMachine.add("GRAB_RACK",
                                   NavigateToAndGrabRack(robot=robot,
                                                         rack_id=RACK_ID,
                                                         rack_navigation_area=RACK_NAVIGATION_AREA),
                                   transitions={"succeeded": "NAVIGATE_TO_FRIDGE",
                                                "failed": "done"})  # What now?

            # Drive to the fridge (waypoint)
            smach.StateMachine.add("NAVIGATE_TO_FRIDGE",
                                   states.NavigateToWaypoint(
                                       robot=robot,
                                       waypoint_designator=ds.EntityByIdDesignator(robot, id=FRIDGE_WAYPOINT)
                                   ),
                                   transitions={"arrived": "SAY_PUT_DRINKS",
                                                "unreachable": "failed",
                                                "goal_not_defined": "failed"}
                                   )

            # Say put drinks in here
            smach.StateMachine.add("SAY_PUT_DRINKS",
                                   states.Say(
                                       robot,
                                       "Hey bartender, have a look at my screen. "
                                       "You can see the orders of the people in the living room",
                                       block=False),
                                   transitions={"spoken": "DRAW_ORDERS_ON_MAP"}
                                   )

            # Display these on the map
            smach.StateMachine.add("DRAW_ORDERS_ON_MAP",
                                   DisplayOrdersOnMap(robot),
                                   transitions={"succeeded": "WAIT_FOR_DRINKS",
                                                "failed": 'WAIT_FOR_DRINKS'})

            # Wait for bartender to put drinks in rack
            smach.StateMachine.add("WAIT_FOR_DRINKS",
                                   states.WaitTime(robot, 40.0),
                                   transitions={"waited": "SAY_DRINKS_COMING",
                                                "preempted": "SAY_DRINKS_COMING"})

            # Say am coming
            smach.StateMachine.add("SAY_DRINKS_COMING",
                                   states.Say(
                                       robot,
                                       "Ladies and gentlemen of the jury, your ice-cold refreshments are on their way",
                                       block=False),
                                   transitions={"spoken": "DELIVER_DRINKS"}
                                   )

            # Deliver the drinks
            smach.StateMachine.add("DELIVER_DRINKS",
                                   states.NavigateToWaypoint(
                                       robot,
                                       waypoint_designator=ds.EntityByIdDesignator(robot, id=WAYPOINT_ID)),
                                   transitions={"arrived": "SAY_DRINKS_ARRIVED",
                                                "unreachable": "failed",
                                                "goal_not_defined": "failed"})

            # Say here are your drinks
            smach.StateMachine.add("SAY_DRINKS_ARRIVED",
                                   states.Say(
                                       robot,
                                       "Ladies and gentlemen,  I have brought you a nice full rack.",
                                       block=False),
                                   transitions={"spoken": "done"}
                                   )
