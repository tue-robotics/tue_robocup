#!/usr/bin/python
import math
import os.path

import numpy as np
import rospkg

from robot_skills.simulation.sim_mode import is_sim_mode
from robot_smach_states.utility import CheckTries, WriteDesignator
import robot_smach_states.util.designators as ds
import robot_smach_states as states
import smach
from challenge_restaurant.ask_take_order import AskTakeTheOrder, AskTakeTheOrderPicoVoice
from challenge_restaurant.get_customer_image import GetCustomerImage
from challenge_restaurant.store_waypoint import StoreWaypoint
from challenge_restaurant.take_orders import TakeOrder, ReciteOrders, ClearOrders
from ed.entity import Entity

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

        image_designator = ds.VariableDesignator(resolve_type=str, name="image")

        if not is_sim_mode():
            reset_tries_des = ds.VariableDesignator(resolve_type=bool, initial_value=False).writeable

        with self:
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
                                       properties={'tags': ['LWave', 'RWave']},
                                       strict=False, nearest=True,
                                       speak=True,
                                       look_range=(-np.pi / 4, np.pi / 4),
                                       look_steps=4,
                                       search_timeout=600),  # 10 minutes
                                   transitions={'found': 'GET_CUSTOMER_IMAGE',
                                                'failed': 'WAIT_FOR_CUSTOMER'})
            #
            # smach.StateMachine.add('SAY_SPEAK',
            #                        states.human_interaction.Say(
            #                            robot,
            #                            "Please make sure that you speak loudly and directly into my microphone "
            #                            "AND SPEAK AFTER THE PING. Like is shown "
            #                            "on my screen", block=True),
            #                        transitions={'spoken': 'SHOW_IMAGE_SPEAK'})
            #
            # smach.StateMachine.add('SHOW_IMAGE_SPEAK',
            #                        states.human_interaction.ShowImageState(
            #                            robot,
            #                            os.path.join(
            #                                rospkg.RosPack().get_path('challenge_restaurant'),
            #                                "images",
            #                                "speak.jpg"
            #                            ),
            #                            seconds=5),
            #                        transitions={'succeeded': 'SAY_I_HAVE_SEEN',
            #                                     'failed': 'SAY_I_HAVE_SEEN'})

            # Asking for confirmation
            smach.StateMachine.add('SAY_I_HAVE_SEEN',
                                   states.human_interaction.Say(
                                       robot, 'I have seen a waving person, should I take the order? '
                                              'Please say "{0} take the order" or "{0} wait"'.format(robot.robot_name)),
                                   transitions={"spoken": 'WAIT_FOR_START'})

            smach.StateMachine.add('GET_CUSTOMER_IMAGE',
                                   GetCustomerImage(robot, customer_designator, image_designator.writeable),
                                   transitions={'succeeded': 'SHOW_CUSTOMER',
                                                'failed': 'SAY_I_HAVE_SEEN'})

            smach.StateMachine.add('SHOW_CUSTOMER',
                                   states.human_interaction.ShowImage(
                                       robot,
                                       image_designator,
                                       duration=30),
                                   transitions={'succeeded': 'SAY_I_HAVE_SEEN',
                                                'failed': 'SAY_I_HAVE_SEEN'})

            if is_sim_mode():
                smach.StateMachine.add('WAIT_FOR_START', AskTakeTheOrder(robot),
                                       transitions={'yes': 'SAY_NAVIGATE_TO_CUSTOMER',
                                                    'wait': 'SAY_WAVING_2',
                                                    'timeout': 'SAY_WAVING_2'})
            else:
                smach.StateMachine.add('WAIT_FOR_START',
                                       WriteDesignator(reset_tries_des, True),
                                       transitions={'written': 'ASK_TAKE_ORDER'})

                smach.StateMachine.add('ASK_TAKE_ORDER', AskTakeTheOrderPicoVoice(robot),
                                       transitions={'yes': 'SAY_NAVIGATE_TO_CUSTOMER',
                                                    'wait': 'SAY_WAVING_2',
                                                    'no_result': 'MAX_TRIES'})
                smach.StateMachine.add('MAX_TRIES',
                                       CheckTries(max_tries=3, reset_des=reset_tries_des),
                                       transitions={'not_yet': 'ASK_TAKE_ORDER',
                                                    'max_tries': 'SAY_WAVING_2'})

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
                                   transitions={'arrived': 'SAY_SPEAK_2',
                                                'unreachable': 'SAY_NAVIGATE_TO_CUSTOMER_FALLBACK',
                                                'goal_not_defined': 'SAY_WAVING_2'})

            smach.StateMachine.add('SAY_SPEAK_2',
                                   states.human_interaction.Say(
                                       robot,
                                       "Hi, please make sure that you speak loudly and directly into my microphone",
                                       block=True),
                                   transitions={'spoken': 'TAKE_ORDER'})

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
                                   transitions={'arrived': 'SAY_SPEAK_2',
                                                'unreachable': 'RETURN_TO_START',
                                                'goal_not_defined': 'RETURN_TO_START'})

            smach.StateMachine.add('SAY_WAVING_2',
                                   states.human_interaction.Say(
                                       robot,
                                       "Mr. Barman, I'm waiting for an order please make sure that the people wave "
                                       "slowly and put their arm up high. Like is shown "
                                       "on my screen", block=True),
                                   transitions={'spoken': 'SHOW_IMAGE_2'})

            smach.StateMachine.add('SHOW_IMAGE_2',
                                   states.human_interaction.ShowImage(
                                       robot,
                                       os.path.join(
                                           rospkg.RosPack().get_path('challenge_restaurant'),
                                           "images",
                                           "waving.jpg"
                                       ),
                                       duration=10),
                                   transitions={'succeeded': 'WAIT_FOR_CUSTOMER',
                                                'failed': 'WAIT_FOR_CUSTOMER'})

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
                                   states.utility.WaitTime(robot=robot, waittime=15.0),
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
                                   states.utility.WaitTime(robot=robot, waittime=15.0),
                                   transitions={'waited': 'RETURN_TO_START',
                                                'preempted': 'STOP'})

            smach.StateMachine.add('RETURN_TO_START',
                                   states.navigation.NavigateToPose(
                                       robot=robot, x=start_x, y=start_y, rz=start_rz, radius=0.3),
                                   transitions={'arrived': 'SAY_WAVING_2',
                                                'unreachable': 'SAY_RETURN_TO_START_FALLBACK',
                                                'goal_not_defined': 'SAY_WAVING_2'})

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
                                   transitions={'arrived': 'SAY_WAVING_2',
                                                'unreachable': 'SAY_WAVING_2',
                                                'goal_not_defined': 'SAY_WAVING_2'})


if __name__ == '__main__':
    from challenge_restaurant.restaurant import Restaurant
    from robot_skills import get_robot
    import sys
    import rospy

    if len(sys.argv) < 2:
        print("Please provide robot name as argument.")
        sys.exit(1)

    rospy.init_node('test_find_emtpy_seat')

    robot_name = sys.argv[1]
    robot = get_robot(robot_name)

    sm = Restaurant(robot)
    sm.execute()
