#!/usr/bin/python
import math
import numpy as np
import os.path

import robot_smach_states as states
import rospkg
import smach
from ed.entity import Entity
from .ask_take_order import AskTakeTheOrder
from .store_waypoint import StoreWaypoint
from .take_orders import TakeOrder, ReciteOrders, ClearOrders

V_TH = 0.5


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
                                                                                       uuid=kitchen_id)
        customer_id = 'current_customer'
        customer_designator = states.util.designators.VariableDesignator(resolve_type=Entity, name=customer_id)
        orders = []

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.utility.Initialize(robot),
                                   transitions={'initialized': 'SAY_WAVING',
                                                'abort': 'STOP'})

            smach.StateMachine.add('SAY_WAVING',
                                   states.human_interaction.Say(
                                       robot,
                                       "Mr. Barman, please make sure that the people wave "
                                       "slowly and put their arm up high. Like is shown "
                                       "on my screen", block=True),
                                   transitions={'spoken': 'SHOW_IMAGE'})

            smach.StateMachine.add('SHOW_IMAGE',
                                   states.human_interaction.ShowImage(
                                       robot,
                                       os.path.join(
                                           rospkg.RosPack().get_path('challenge_restaurant'),
                                           "images",
                                           "waving.jpg"
                                       ),
                                       duration=10),
                                   transitions={'succeeded': 'STORE_KITCHEN',
                                                'failed': 'STORE_KITCHEN'})

            smach.StateMachine.add('STORE_KITCHEN',
                                   StoreWaypoint(robot=robot, location_id=kitchen_id),
                                   transitions={'done': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('WAIT_FOR_CUSTOMER',
                                   states.human_interaction.FindFirstPerson(
                                       robot, customer_designator.writeable,
                                       properties={'tags': ['LWaving', 'RWaving']},
                                       strict=False, nearest=True,
                                       speak=True,
                                       look_range=(-np.pi / 4, np.pi / 4),
                                       look_steps=4,
                                       search_timeout=600),  # 10 minutes
                                   transitions={'found': 'SAY_I_HAVE_SEEN',
                                                'failed': 'WAIT_FOR_CUSTOMER'})

            # Asking for confirmation
            smach.StateMachine.add('SAY_I_HAVE_SEEN',
                                   states.human_interaction.Say(
                                       robot, 'I have seen a waving person, should I take the order? '
                                              'Please say "{0} take the order" or "{0} wait"'.format(robot.robot_name)),
                                   transitions={"spoken": 'WAIT_FOR_START'})

            smach.StateMachine.add('WAIT_FOR_START', AskTakeTheOrder(robot),
                                   transitions={'yes': 'SAY_NAVIGATE_TO_CUSTOMER',
                                                'wait': 'WAIT_FOR_CUSTOMER',
                                                'timeout': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('SAY_NAVIGATE_TO_CUSTOMER',
                                   states.human_interaction.Say(
                                       robot,
                                       "I am at your service, I will be there shortly! Coming your way my amigo!",
                                       block=True),
                                   transitions={'spoken': 'NAVIGATE_TO_CUSTOMER'})
            # End Asking for confirmation

            smach.StateMachine.add('NAVIGATE_TO_CUSTOMER',
                                   states.navigation.NavigateToObserve(
                                       robot=robot, entity_designator=customer_designator,
                                       radius=0.8),
                                   transitions={'arrived': 'TAKE_ORDER',
                                                'unreachable': 'SAY_NAVIGATE_TO_CUSTOMER_FALLBACK',
                                                'goal_not_defined': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('SAY_NAVIGATE_TO_CUSTOMER_FALLBACK',
                                   states.human_interaction.Say(
                                       robot, "Help, lets try it another way"
                                   ),
                                   transitions={'spoken': 'TURN_AROUND'})

            smach.StateMachine.add('TURN_AROUND',
                                   states.navigation.ForceDrive(robot, 0, 0, V_TH, (2 * math.pi) / V_TH),
                                   transitions={'done': 'NAVIGATE_TO_CUSTOMER_FALLBACK'})

            smach.StateMachine.add('NAVIGATE_TO_CUSTOMER_FALLBACK',
                                   states.navigation.NavigateToObserve(
                                       robot=robot, entity_designator=customer_designator,
                                       radius=1.1),
                                   transitions={'arrived': 'TAKE_ORDER',
                                                'unreachable': 'RETURN_TO_START',
                                                'goal_not_defined': 'RETURN_TO_START'})

            smach.StateMachine.add('TAKE_ORDER',
                                   TakeOrder(robot=robot, entity_designator=customer_designator, orders=orders),
                                   transitions={'succeeded': 'NAVIGATE_TO_KITCHEN',
                                                'failed': 'RETURN_TO_START'})

            smach.StateMachine.add('NAVIGATE_TO_KITCHEN',
                                   states.navigation.NavigateToWaypoint(
                                       robot=robot, waypoint_designator=kitchen_designator,
                                       radius=0.15),
                                   transitions={'arrived': 'RECITE_ORDER',
                                                'unreachable': 'SAY_NAVIGATE_TO_KITCHEN_FALLBACK',
                                                'goal_not_defined': 'SAY_NAVIGATE_TO_KITCHEN_FALLBACK'})

            smach.StateMachine.add('SAY_NAVIGATE_TO_KITCHEN_FALLBACK',
                                   states.human_interaction.Say(robot, "Help, how do I get there?", block=False),
                                   transitions={'spoken': 'TURN_AROUND_KITCHEN_FALLBACK'})

            smach.StateMachine.add('TURN_AROUND_KITCHEN_FALLBACK',
                                   states.navigation.ForceDrive(robot, 0, 0, V_TH, math.pi / V_TH),
                                   transitions={'done': 'NAVIGATE_TO_KITCHEN_FALLBACK'})

            smach.StateMachine.add('NAVIGATE_TO_KITCHEN_FALLBACK',
                                   states.navigation.NavigateToWaypoint(
                                       robot=robot, waypoint_designator=kitchen_designator,
                                       radius=0.20),
                                   transitions={'arrived': 'RECITE_ORDER',
                                                'unreachable': 'SAY_NAVIGATE_TO_KITCHEN_FALLBACK',
                                                'goal_not_defined': 'SAY_NAVIGATE_TO_KITCHEN_FALLBACK'})

            smach.StateMachine.add('RECITE_ORDER',
                                   ReciteOrders(robot=robot, orders=orders),
                                   transitions={'spoken': 'CLEAR_ORDER'})

            smach.StateMachine.add('CLEAR_ORDER',
                                   ClearOrders(orders=orders),
                                   transitions={'succeeded': 'SAY_CANNOT_GRASP'})

            smach.StateMachine.add('SAY_CANNOT_GRASP',
                                   states.human_interaction.Say(robot, "I am unable to grasp my own order, "
                                                                       "could you please put it in my basket"),
                                   transitions={'spoken': 'WAIT_FOR_OBJECTS'})

            smach.StateMachine.add('WAIT_FOR_OBJECTS',
                                   states.utility.WaitTime(robot=robot, waittime=10.0),
                                   transitions={'waited': 'BRING_OBJECTS',
                                                'preempted': 'STOP'})

            smach.StateMachine.add('BRING_OBJECTS',
                                   states.navigation.NavigateToObserve(
                                       robot=robot, entity_designator=customer_designator,
                                       radius=1.1),
                                   transitions={'arrived': 'SAY_OBJECTS',
                                                'unreachable': 'SAY_BRING_OBJECTS_FALLBACK',
                                                'goal_not_defined': 'RETURN_TO_START'})

            smach.StateMachine.add('SAY_BRING_OBJECTS_FALLBACK',
                                   states.human_interaction.Say(robot, "Help, how do I get there?"),
                                   transitions={'spoken': 'TURN_AROUND_BRING_OBJECTS_FALLBACK'})

            smach.StateMachine.add('TURN_AROUND_BRING_OBJECTS_FALLBACK',
                                   states.navigation.ForceDrive(robot, 0, 0, V_TH, (2 * math.pi) / V_TH),
                                   transitions={'done': 'BRING_OBJECTS_FALLBACK'})

            smach.StateMachine.add('BRING_OBJECTS_FALLBACK',
                                   states.navigation.NavigateToObserve(
                                       robot=robot, entity_designator=customer_designator,
                                       radius=1.1),
                                   transitions={'arrived': 'SAY_OBJECTS',
                                                'unreachable': 'SAY_OBJECTS',
                                                'goal_not_defined': 'RETURN_TO_START'})

            smach.StateMachine.add('SAY_OBJECTS',
                                   states.human_interaction.Say(robot, "Hi there handsome, here are your objects, "
                                                                       "please take them from my basket"),
                                   transitions={'spoken': 'WAIT_TO_TAKE_OBJECTS'})

            smach.StateMachine.add('WAIT_TO_TAKE_OBJECTS',
                                   states.utility.WaitTime(robot=robot, waittime=10.0),
                                   transitions={'waited': 'RETURN_TO_START',
                                                'preempted': 'STOP'})

            smach.StateMachine.add('RETURN_TO_START',
                                   states.navigation.NavigateToPose(
                                       robot=robot, x=start_x, y=start_y, rz=start_rz, radius=0.3),
                                   transitions={'arrived': 'WAIT_FOR_CUSTOMER',
                                                'unreachable': 'SAY_RETURN_TO_START_FALLBACK',
                                                'goal_not_defined': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('SAY_RETURN_TO_START_FALLBACK',
                                   states.human_interaction.Say(robot, "Help, how do I get back?"),
                                   transitions={'spoken': 'RETURN_TO_START_TURN_AROUND'})

            smach.StateMachine.add('RETURN_TO_START_TURN_AROUND',
                                   states.navigation.ForceDrive(robot, 0, 0, V_TH, math.pi / V_TH),
                                   transitions={'done': 'RETURN_TO_START_FALLBACK'})

            smach.StateMachine.add('RETURN_TO_START_FALLBACK',
                                   states.navigation.NavigateToObserve(
                                       robot=robot, entity_designator=customer_designator,
                                       radius=0.7),
                                   transitions={'arrived': 'WAIT_FOR_CUSTOMER',
                                                'unreachable': 'WAIT_FOR_CUSTOMER',
                                                'goal_not_defined': 'WAIT_FOR_CUSTOMER'})
