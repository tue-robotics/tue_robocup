#!/usr/bin/python

# ROS
import math
import smach

# TU/e Robotics
import robot_smach_states as states


# Challenge restaurant
from store_waypoint import StoreWaypoint
from take_orders import TakeOrder, ReciteOrders
from wait_for_customer import WaitForCustomer


class Restaurant(smach.StateMachine):
    """ Main statemachine for the restaurant challenge """
    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=['STOP'])

        start_pose = robot.base.get_location()
        start_x = start_pose.frame.p.x()
        start_y = start_pose.frame.p.y()
        start_rz = start_pose.frame.M.GetRPY()[2]

        kitchen_id = "kitchen"
        kitchen_designator = states.util.designators.ed_designators.EdEntityDesignator(robot=robot,
                                                                                       id=kitchen_id)

        caller_id = "customer"
        caller_designator = states.util.designators.ed_designators.EdEntityDesignator(robot=robot,
                                                                                      id=caller_id)

        orders = {}

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'STORE_KITCHEN',
                                                'abort': 'STOP'})

            smach.StateMachine.add('STORE_KITCHEN',
                                   StoreWaypoint(robot=robot, location_id=kitchen_id),
                                   transitions={'done': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('WAIT_FOR_CUSTOMER',
                                   WaitForCustomer(robot, caller_id, kitchen_designator),
                                   transitions={'succeeded': 'NAVIGATE_TO_CUSTOMER',
                                                'aborted': 'STOP',
                                                'rejected': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('NAVIGATE_TO_CUSTOMER',
                                   states.NavigateToObserve(robot=robot, entity_designator=caller_designator,
                                                            radius=0.85),
                                   transitions={'arrived': 'TAKE_ORDER',
                                                'unreachable': 'SAY_NAVIGATE_TO_CUSTOMER_FALLBACK',
                                                'goal_not_defined': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('SAY_NAVIGATE_TO_CUSTOMER_FALLBACK',
                                   states.Say(robot, "Help, how do I get there?"),
                                   transitions={'spoken': 'TURN_AROUND'})

            smach.StateMachine.add('TURN_AROUND',
                                   states.Turn(robot, radians=2*math.pi),
                                   transitions={'turned': 'NAVIGATE_TO_CUSTOMER_FALLBACK'})

            smach.StateMachine.add('NAVIGATE_TO_CUSTOMER_FALLBACK',
                                   states.NavigateToObserve(robot=robot, entity_designator=caller_designator,
                                                            radius=0.85),
                                   transitions={'arrived': 'TAKE_ORDER',
                                                'unreachable': 'RETURN_TO_START',
                                                'goal_not_defined': 'RETURN_TO_START'})

            smach.StateMachine.add('TAKE_ORDER',
                                   TakeOrder(robot=robot, location=caller_id, orders=orders),
                                   transitions={'succeeded': 'NAVIGATE_TO_KITCHEN',
                                                'failed': 'RETURN_TO_START'})

            smach.StateMachine.add('NAVIGATE_TO_KITCHEN',
                                   states.NavigateToWaypoint(robot=robot, waypoint_designator=kitchen_designator,
                                                             radius=0.15),
                                   transitions={'arrived': 'RECITE_ORDER',
                                                'unreachable': 'SAY_NAVIGATE_TO_KITCHEN_FALLBACK',
                                                'goal_not_defined': 'SAY_NAVIGATE_TO_KITCHEN_FALLBACK'})

            smach.StateMachine.add('SAY_NAVIGATE_TO_KITCHEN_FALLBACK',
                                   states.Say(robot, "Help, how do I get there?"),
                                   transitions={'spoken': 'TURN_AROUND_KITCHEN_FALLBACK'})

            smach.StateMachine.add('TURN_AROUND_KITCHEN_FALLBACK',
                                   states.Turn(robot, radians=math.pi),
                                   transitions={'turned': 'NAVIGATE_TO_KITCHEN_FALLBACK'})

            smach.StateMachine.add('NAVIGATE_TO_KITCHEN_FALLBACK',
                                   states.NavigateToWaypoint(robot=robot, waypoint_designator=kitchen_designator,
                                                             radius=0.20),
                                   transitions={'arrived': 'RECITE_ORDER',
                                                'unreachable': 'RECITE_ORDER',
                                                'goal_not_defined': 'RECITE_ORDER'})

            smach.StateMachine.add('RECITE_ORDER',
                                   ReciteOrders(robot=robot, orders=orders),
                                   transitions={'spoken': 'SAY_CANNOT_GRASP'})

            smach.StateMachine.add('SAY_CANNOT_GRASP',
                                   states.Say(robot, "I am unable to grasp my own order,"
                                                     "could you please put it in my basket"),
                                   transitions={'spoken': 'WAIT_FOR_OBJECTS'})

            smach.StateMachine.add('WAIT_FOR_OBJECTS',
                                   states.WaitTime(robot=robot, waittime=5.0),
                                   transitions={'waited': 'BRING_OBJECTS',
                                                'preempted': 'STOP'})

            smach.StateMachine.add('BRING_OBJECTS',
                                   states.NavigateToObserve(robot=robot, entity_designator=caller_designator,
                                                            radius=0.85),
                                   transitions={'arrived': 'SAY_OBJECTS',
                                                'unreachable': 'SAY_BRING_OBJECTS_FALLBACK',
                                                'goal_not_defined': 'RETURN_TO_START'})

            smach.StateMachine.add('SAY_BRING_OBJECTS_FALLBACK',
                                   states.Say(robot, "Help, how do I get there?"),
                                   transitions={'spoken': 'TURN_AROUND_BRING_OBJECTS_FALLBACK'})

            smach.StateMachine.add('TURN_AROUND_BRING_OBJECTS_FALLBACK',
                                   states.Turn(robot, radians=2 * math.pi),
                                   transitions={'turned': 'BRING_OBJECTS_FALLBACK'})

            smach.StateMachine.add('BRING_OBJECTS_FALLBACK',
                                   states.NavigateToObserve(robot=robot, entity_designator=caller_designator,
                                                            radius=0.85),
                                   transitions={'arrived': 'SAY_OBJECTS',
                                                'unreachable': 'SAY_OBJECTS',
                                                'goal_not_defined': 'RETURN_TO_START'})

            smach.StateMachine.add('SAY_OBJECTS',
                                   states.Say(robot, "Dear mister, here are your objects, "
                                                     "please take them from my basket"),
                                   transitions={'spoken': 'WAIT_TO_TAKE_OBJECTS'})

            smach.StateMachine.add('WAIT_TO_TAKE_OBJECTS',
                                   states.WaitTime(robot=robot, waittime=5.0),
                                   transitions={'waited': 'RETURN_TO_START',
                                                'preempted': 'STOP'})

            smach.StateMachine.add('RETURN_TO_START',
                                   states.NavigateToPose(robot=robot, x=start_x, y=start_y, rz=start_rz, radius=0.3),
                                   transitions={'arrived': 'WAIT_FOR_CUSTOMER',
                                                'unreachable': 'SAY_RETURN_TO_START_FALLBACK',
                                                'goal_not_defined': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('SAY_RETURN_TO_START_FALLBACK',
                                   states.Say(robot, "Help, how do I get back?"),
                                   transitions={'spoken': 'RETURN_TO_START_TURN_AROUND'})

            smach.StateMachine.add('RETURN_TO_START_TURN_AROUND',
                                   states.Turn(robot, radians=math.pi),
                                   transitions={'turned': 'RETURN_TO_START_FALLBACK'})

            smach.StateMachine.add('RETURN_TO_START_FALLBACK',
                                   states.NavigateToObserve(robot=robot, entity_designator=caller_designator,
                                                            radius=0.7),
                                   transitions={'arrived': 'WAIT_FOR_CUSTOMER',
                                                'unreachable': 'WAIT_FOR_CUSTOMER',
                                                'goal_not_defined': 'WAIT_FOR_CUSTOMER'})
