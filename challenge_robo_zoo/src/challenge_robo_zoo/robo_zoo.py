#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cleanup')
import rospy

import smach

import robot_smach_states as states
from robot_smach_states.util.startup import startup

from psi import Compound, Conjunction, Sequence


""" TODOs, BUGs:
- Define point_of_interest for:
    ordering_table,
    pickup_table,
    storage_table
- Define dropoff_point for trash_bin
"""

class RoboZoo(smach.StateMachine):
    """The goal of the challenge is to attract people and be attractive to an audience.
    So, the robot will hand out cans of coke and fanta on one end of a table and clean up empty cans at another end.
    The robot will get the full cans from another table, at the back of the demonstration-stand.

    Optionnally, we can also have to robot ask at the stand what kind of drink to get for a waiting user.

    So, the challenge consists of three parts:
    0:  (Optionally) Asking what drink a 'customer' wants.
        0.0:    NavigateGeneric(query_ordering_table)
        0.1:    Ask the customer what drink he wants. 
                We should have two very different sounding drinks, like coke and fanta etc.
                Speech is tricky, especially in the expected environment.
        0.2:    Assert the ordered drink to the world model
    1:  Getting the (ordered) drink from a table. 
        This involves two states: 
        1.1:    NavigateGeneric(query_storage_table)
        1.2:    GrabMachine(query_ordered_drink)
    2:  Brink the drink to the table and place it on the table. Or directly hand it over to the customer?
        2.1:    NavigateGeneric(query_ordering_table)
        2.2:    PlaceObject(query_ordering_table)
    3:  Then we look if there are any cans delivered to the other end of the table, and clean them up if so.
        3.1:    NavigateGeneric(query_pickup_table)
        3.2:    GrabMachine(query_any_can)
        3.3:    NavigateGeneric(query_trashbin)
        3.4:    Dropoff(query_trashbin).

    So, we have these queries:
    - query_storage_table:  The table where drinks are originally stored.
    - query_ordered_drink:  Which drink the customer wants, so what we have to look for. 
                            We assert some variable to this as well when we get the order
    - query_ordering_table: The table where the customers are at; where we ask for an order and deliver it to.
    - query_pickup_table:   The table where customers can leave their empty cans for the robot to pick up.
    - query_any_can:        At the pickup_table, look for any sort of can, instead of a specific can.
    - query_trashbin:       Location of the trashbin.
    """

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])

        self.init_knowledge()

        query_storage_table =   Compound("point_of_interest", "storage_table", Compound("point_3d", "X", "Y", "Z"))

        #To store an order, assertz Compound("goal", Compound("serve", "Drink")) to the world model
        query_ordered_drink =   Conjunction(
                                    Compound("goal", Compound("serve", "Drink")),
                                    Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                    Compound( "property_expected", "ObjectID", "position", 
                                        Compound("in_front_of", "amigo")))
        
        query_ordering_table =  Compound("point_of_interest", "ordering_table", Compound("point_3d", "X", "Y", "Z"))
        
        query_pickup_table =    Compound("point_of_interest", "pickup_table", Compound("point_3d", "X", "Y", "Z"))
        
        query_any_can =         Conjunction(
                                    Compound( "property_expected", "ObjectID", "class_label", "can"),
                                    Compound( "property_expected", "ObjectID", "position", 
                                        Compound("in_front_of", "amigo")))
        
        query_trashbin =        Compound("dropoff_point", "trash_bin", Compound("point_3d", "X", "Y", "Z"))

        with self:
            smach.StateMachine.add( "SET_CURRENT_DRINK",
                                    smach.CBState(self.set_serve_drink, cb_kwargs={'drink':'coke'},
                                    transitions={"asserted"             :"GOTO_STORAGE"}))

            smach.StateMachine.add( "GOTO_STORAGE",
                                    states.NavigateGeneric(robot, lookat_query=query_storage_table),
                                    transitions={   "arrived"           :"LOOK_FOR_DRINK", 
                                                    "unreachable"       :"LOOK_FOR_DRINK", 
                                                    "preempted"         :"Aborted", 
                                                    "goal_not_defined"  :"Failed"})

            smach.StateMachine.add( "LOOK_FOR_DRINK",
                                    states.LookForObjectsAtROI(robot, query_storage_table, query_ordered_drink),
                                    transitions={   'looking'           :'LOOK_FOR_DRINK',
                                                    'object_found'      :'GRAB_DRINK',
                                                    'no_object_found'   :'GOTO_STORAGE', #TODO: Not the best option maybe
                                                    'abort'             :'Aborted'})

            smach.StateMachine.add( "GRAB_DRINK",
                                    states.GrabMachine(robot, "left", query_ordered_drink),
                                    transitions={   'succeeded'         :'GOTO_ORDERING',
                                                    'failed'            :'LOOK_FOR_DRINK' }) #TODO: Or ask for help

            smach.StateMachine.add( "GOTO_ORDERING",
                                    states.NavigateGeneric(robot, lookat_query=query_ordering_table),
                                    transitions={   "arrived"           :"PLACE_ORDER", 
                                                    "unreachable"       :"PLACE_ORDER", #TODO: we should ask for help
                                                    "preempted"         :"Aborted", 
                                                    "goal_not_defined"  :"Failed"})

            smach.StateMachine.add( "PLACE_ORDER",
                                    states.PlaceObject(robot, "left", placement_query=query_ordering_table),
                                    transitions={   "succeeded"         :"GOTO_PICKUP",
                                                    "failed"            :"PLACE_ORDER",     #TODO: Ask for help
                                                    "target_lost"       :"PLACE_ORDER"})    #TODO: Ask for help

            smach.StateMachine.add( "GOTO_PICKUP",
                                    states.NavigateGeneric(robot, lookat_query=query_pickup_table),
                                    transitions={   "arrived"           :"LOOK_FOR_EMPTY_CAN", 
                                                    "unreachable"       :"LOOK_FOR_EMPTY_CAN", 
                                                    "preempted"         :"Aborted", 
                                                    "goal_not_defined"  :"Failed"})

            smach.StateMachine.add( "LOOK_FOR_EMPTY_CAN",
                                    states.LookForObjectsAtROI(robot, query_pickup_table, query_any_can),
                                    transitions={   'looking'           :'LOOK_FOR_EMPTY_CAN',
                                                    'object_found'      :'GRAB_EMPTY_CAN',
                                                    'no_object_found'   :'SET_CURRENT_DRINK', #Nothing here, so skip this step
                                                    'abort'             :'Aborted'})

            smach.StateMachine.add( "GRAB_EMPTY_CAN",
                                    states.GrabMachine(robot, "left", query_any_can),
                                    transitions={   'succeeded'         :'GOTO_TRASHBIN',
                                                    'failed'            :'SET_CURRENT_DRINK' }) #TODO: or ask for help

            smach.StateMachine.add( "GOTO_TRASHBIN",
                                    states.NavigateGeneric(robot, lookat_query=query_trashbin),
                                    transitions={   "arrived"           :"DROPOFF_EMPTY_CAN", 
                                                    "unreachable"       :"DROPOFF_EMPTY_CAN", 
                                                    "preempted"         :"Aborted", 
                                                    "goal_not_defined"  :"Failed"})

            smach.StateMachine.add("DROPOFF_EMPTY_CAN",
                                    states.DropObject("left", robot, query_trashbin),
                                    transitions={   'succeeded'         :'SET_CURRENT_DRINK',
                                                    'failed'            :'SET_CURRENT_DRINK',
                                                    'target_lost'       :'SET_CURRENT_DRINK'})


    def init_knowledge(self):
        self.robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
        self.robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("disposed", "X")))
        
        self.robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        self.robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
    
        self.robot.reasoner.assertz(Compound("challenge", "robo_zoo"))

    def set_serve_drink(self, drink="coke"):
        self.robot.reasoner.query(Compound("retractall", Compound("goal", Compound("serve", "Drink"))))
        self.robot.reasoner.assertz(Compound("goal", Compound("serve", drink)))
        return "asserted"


if __name__ == "__main__":
    rospy.init_node("challenge_robo_zoo")

    startup(RoboZoo)