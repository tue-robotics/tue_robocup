import os
import cv2
import numpy as np
from datetime import datetime

# ROS
import rospy
import smach
import cv_bridge
import rospkg

# TU/e Robotics
import robot_smach_states as states
from robot_skills import get_robot_from_argv
from challenge_final.select_option_for_image import SelectOptionForImage
from challenge_final.display_orders_on_map import DisplayOrdersOnMap

DRINK_OPTIONS = ['beer', 'coke', 'water', 'energy']
DEFAULT_ORDER = 'beer'

detected_person_index = 0


class GetOrders(smach.StateMachine):
    def __init__(self, robot):
        """
        Gets the orders of the detected people using the Telegram interface.

        * Telegram interaction
        * Drive to operator
        * Display order on screen

        Input keys:
        * detected_people: a list of {'rgb':..., 'person_detection':..., 'map_ps':...}-dictionaries

        Output keys:
        * orders: detected people: list with updated dicts (orders have been added)

        :param robot: (Robot) api object
        """
        smach.StateMachine.__init__(self,
                                    outcomes=["done"],
                                    input_keys=["detected_people"],
                                    output_keys=["detected_people"])

        with self:
            @smach.cb_interface(outcomes=["next", "stop_iteration"],
                                input_keys=['detected_people'],
                                output_keys=['person_dict'])
            def iterate_next_person(user_data):
                global detected_person_index
                try:
                    rospy.loginfo("Iterating to person {}/{}"
                                  .format(detected_person_index, len(user_data['detected_people'])))
                    user_data['person_dict'] = user_data['detected_people'][detected_person_index]
                    return 'next'
                except IndexError:
                    detected_person_index = 0
                    return 'stop_iteration'
            smach.StateMachine.add('ITERATE_NEXT_PERSON',
                                   smach.CBState(iterate_next_person),
                                   transitions={'next': 'SELECT_ORDER_FOR_IMAGE',
                                                'stop_iteration': 'done'})

            smach.StateMachine.add("SELECT_ORDER_FOR_IMAGE",
                                   SelectOptionForImage(robot,
                                                        question='What do you want to drink?',
                                                        options=DRINK_OPTIONS,
                                                        instruction='If you are in the image, please pick an option from: '),
                                   transitions={"succeeded": "STORE_ORDER",
                                                "failed": 'SAY_ORDERED_TOO_LATE'})

            smach.StateMachine.add('SAY_ORDERED_TOO_LATE',
                                   states.Say(robot, ["I didn't get an order in time. "
                                                      "Probably can't get online, so I'll just try to get you all a {}".format(DEFAULT_ORDER)],
                                              block=False),
                                   transitions={'spoken': 'ASSIGN_RANDOM_DRINKS'})

            # Here comes Loys stuff (stuff is passed)
            @smach.cb_interface(outcomes=["done"],
                                input_keys=['detected_people', 'selection'],
                                output_keys=['detected_people'])
            def assign_random_drinks(user_data):
                """
                Edits user_data['detected_people'] elements (which are dicts) and adds a new 'selection'-key

                :param user_data:
                :return:
                """
                global detected_person_index
                # import ipdb; ipdb.set_trace()
                selection = DEFAULT_ORDER
                for i, elem in enumerate(user_data['detected_people']):
                    try:
                        if 'selection' not in user_data['detected_people'][i]:
                            rospy.loginfo("Storing order for person {}/{}: {}".format(i,
                                                                                      len(user_data['detected_people']),
                                                                                      selection))
                            user_data['detected_people'][i]['selection'] = selection
                    except Exception as e:
                        rospy.logerr('Cannot store order somehow'.format(e))

                return 'done'
            smach.StateMachine.add('ASSIGN_RANDOM_DRINKS',
                                   smach.CBState(assign_random_drinks),
                                   transitions={'done': 'done'})

            # Here comes Loys stuff (stuff is passed)
            @smach.cb_interface(outcomes=["done", 'failed'],
                                input_keys=['detected_people', 'selection'],
                                output_keys=['detected_people'])
            def store_current_person_order(user_data):
                """
                Edits user_data['detected_people'] elements (which are dicts) and adds a new 'selection'-key

                :param user_data:
                :return:
                """
                global detected_person_index
                # import ipdb; ipdb.set_trace()
                try:
                    rospy.loginfo("Storing order for person {}/{}: {}"
                                  .format(detected_person_index,
                                          len(user_data['detected_people']),
                                          user_data['selection']))
                    user_data['detected_people'][detected_person_index]['selection'] = user_data['selection']
                    return 'done'
                except Exception as e:
                    rospy.logerr('Cannot store order somehow'.format(e))
                    return 'failed'
            smach.StateMachine.add('STORE_ORDER',
                                   smach.CBState(store_current_person_order),
                                   transitions={'done': 'INCREASE',
                                                'failed': 'done'})

            @smach.cb_interface(outcomes=["done"])
            def increase_ppl_counter(user_data):
                """
                Increments number of detected people.

                :param user_data:
                :return:
                """
                global detected_person_index
                detected_person_index += 1
                return 'done'
            smach.StateMachine.add('INCREASE',
                                   smach.CBState(increase_ppl_counter),
                                   transitions={'done': 'ITERATE_NEXT_PERSON'})

            # smach.StateMachine.add("DRAW_ORDERS_ON_MAP",
            #                        DisplayOrdersOnMap(robot),
            #                        transitions={"succeeded": "done",
            #                                     "failed": 'done'})


if __name__ == "__main__":

    rospy.init_node("test_get_orders")
    rospy.sleep(3)

    import mock
    # Robot
    # _robot = get_robot_from_argv(index=1)
    _robot = mock.MagicMock()
    _robot.speech = mock.MagicMock()
    def speak(*args, **kwargs):
        print(args, kwargs)
    _robot.speech.speak = speak

    sm = GetOrders(robot=_robot)

    import sys
    import pickle
    import random
    ppl_dicts = pickle.load(open(sys.argv[2]))
    # ppl_dicts is a list of dicts {'rgb':sensor_msgs/Image, 'person_detection':..., 'map_ps':...}

    # Test data
    # ToDo: load Reins pickled file here
    user_data = smach.UserData()

    random.shuffle(ppl_dicts)
    user_data['detected_people'] = ppl_dicts[:4]
    print(sm.execute(user_data))

    print([person_dict['selection'] for person_dict in user_data['detected_people']])


