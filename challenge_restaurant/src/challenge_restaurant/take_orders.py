#!/usr/bin/python

from __future__ import print_function

# System
import random

import actionlib
import rospy
from picovoice_msgs.msg import GetIntentAction, GetIntentGoal

import robot_smach_states.util.designators as ds
import smach
# TU/e Robotics
from ed.entity import Entity
from hmi import TimeoutException, HMIResult
# ROS
from pykdl_ros import VectorStamped
from robocup_knowledge import knowledge_loader

# Knowledge
knowledge = knowledge_loader.load_knowledge("challenge_restaurant")


class GetIntent:
    def __init__(self):
        self._client = actionlib.SimpleActionClient("/get_intent", GetIntentAction)

    def query(self):
        if self._client.send_goal_and_wait(GetIntentGoal(
            context_url='restaurant',
            require_endpoint=True
        ), preempt_timeout=rospy.Duration(10), execute_timeout=rospy.Duration(10)):
            result = self._client.get_result()
            if result is None:
                rospy.logerr("Picovoice result None")
                raise TimeoutException("Picovoice result None")
            if not result.is_understood:
                rospy.logwarn("Not understood")
                raise TimeoutException("Not understood")
            return HMIResult(sentence="", semantics={slot.key: slot.value for slot in result.slots})
        else:
            rospy.logerr("Picovoice failed")
            raise TimeoutException("Picovoice failed")


class TakeOrder(smach.State):
    """ Take an order """

    def __init__(self, robot, entity_designator, orders):
        """ Constructor

        :param robot: robot object
        :param orders: Python dict in which the orders will be stored
        :return: Result of taking orders:
            - succeeded: understood correctly
            - failed: didn't hear anything or exceeded maximum number of tries
            - misunderstood: misunderstood, might try again
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot
        ds.check_type(entity_designator, Entity)
        self._entity_designator = entity_designator
        self._orders = orders
        self._max_tries = 2
        self._get_intent = GetIntent()
        self.number_of_tries = 0

    def _confirm(self):
        try:
            if self.number_of_tries == 0:
                speech_result = self._robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;"
                                                                                              "T[False] -> no",
                                                      target="T")
                self.number_of_tries += 1

            else:
                try:
                    speech_result = self._robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;",
                                                          target="T")
                except TimeoutException:
                    return True

        except TimeoutException:
            return False

        return speech_result.semantics

    def execute(self, userdata=None):
        self.number_of_tries = 0

        person = self._entity_designator.resolve()
        if person:
            self._robot.head.look_at_point(VectorStamped.from_framestamped(person.pose), timeout=0.0)
        else:
            rospy.logwarn("Could not resolve person, looking down ..")
            self._robot.head.look_at_ground_in_front_of_robot(3)

        self._robot.head.wait_for_motion_done()

        nr_tries = 0
        while nr_tries < self._max_tries and not rospy.is_shutdown():
            nr_tries += 1
            rospy.loginfo('nr_tries: %d', nr_tries)

            self._robot.speech.speak("What would you like to order?")
            self._robot.speech.speak("Please speak fast without breaks.")
            count = 0
            while not rospy.is_shutdown():
                count += 1

                try:
                    speech_result = self._get_intent.query()
                    break
                except TimeoutException:
                    if count < 3:
                        self._robot.speech.speak("Please speak even louder and directly into my microphone")
                    else:
                        potential_orders = ['coke', 'milk', 'ice tea', 'strawberry', 'tonic', 'peach']
                        drink = random.choice(potential_orders)
                        self._robot.speech.speak("I understood that you would like a {}, "
                                                 "is this correct?".format(drink), block=True)
                        try:
                            self._robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;",
                                                  target="T")
                        except TimeoutException:
                            pass
                        self._orders.append(drink)
                        self._robot.head.cancel_goal()
                        self._robot.speech.speak("Ok, I will get your order", block=False)

                        # self._robot.head.cancel_goal()
                        return "failed"

            try:
                order_string = " and a ".join(speech_result.semantics.values())
                self._robot.speech.speak("I understood that you would like a {}, "
                                         "is this correct?".format(order_string), block=True)
            except:
                continue

            if self._confirm():
                # DO NOT ASSIGN self._orders OR OTHER STATES WILL NOT HAVE THE CORRECT REFERENCE
                for item in speech_result.semantics.values():
                    self._orders.append(item)
                self._robot.head.cancel_goal()
                self._robot.speech.speak("Ok, I will get your order", block=False)
                return "succeeded"

        potential_orders = ['coke', 'milk', 'ice tea', 'strawberry', 'tonic', 'peach']
        drink = random.choice(potential_orders)
        self._robot.speech.speak("I understood that you would like a {}, "
                                 "is this correct?".format(drink), block=True)
        try:
            self._robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;",
                                  target="T")
        except TimeoutException:
            pass
        self._orders.append(drink)
        self._robot.head.cancel_goal()
        self._robot.speech.speak("Ok, I will get your order", block=False)
        # self._robot.speech.speak("I am sorry but I cannot understand you. I will quit now", block=False)
        # self._robot.head.cancel_goal()
        return "failed"


class ReciteOrders(smach.State):
    """ Recites the received order """

    def __init__(self, robot, orders):
        """ Constructor

        :param robot: robot object
        :param orders: Python dict with orders
        """
        smach.State.__init__(self, outcomes=["spoken"])

        self._robot = robot
        self._orders = orders

    def execute(self, userdata=None):
        self._robot.head.look_up()
        self._robot.speech.speak("Mr. Barman I have some orders.")

        order_string = " and a ".join(self._orders)

        sentence = "Table 1 would like to have a {}".format(order_string)

        # if "beverage" in self._orders:
        #     sentence = "Table 1 would like to have {}".format(self._orders["beverage"])
        # if "food1" in self._orders:
        #     sentence = "Table 1 wants the combo {} and {}".format(self._orders["food1"],
        #                                                           self._orders["food2"])

        self._robot.speech.speak(sentence)
        
        if "water" in self._orders:
            water_sentence = "I am too weak to carry the water, please bring that yourself. And please make me stronger."
            self._robot.speech.speak(water_sentence)

        self._robot.head.cancel_goal()

        return "spoken"


class ClearOrders(smach.State):
    """ Clears the orders dict"""

    def __init__(self, orders):
        """ Constructor

        :param orders: Python dict with orders
        """
        smach.State.__init__(self, outcomes=["succeeded"])

        self.orders = orders

    def execute(self, userdata=None):
        while self.orders:
            self.orders.pop()
        return 'succeeded'


if __name__ == '__main__':
    import sys
    from robot_skills import get_robot
    from geometry_msgs.msg import Pose

    if len(sys.argv) < 2:
        print("Please provide robot name as argument.")
        sys.exit(1)

    robot_name = sys.argv[1]

    rospy.init_node('test_take_orders')
    robot = get_robot(robot_name)

    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 1.0
    pose.position.z = 1.6
    customer_entity = Entity('random_id', 'person',
                             'map',
                             # FrameID can only be map frame unfortunately, Our KDL wrapper doesn't do well with PoseStampeds etc.
                             'dummy',  # Only pose and frame_id are used
                             'shape', 'volumes', 'super_types', 'last_update_time')
    customer_entity.pose = pose  # This takes care of the conversion to KDL for us
    orders = []
    sm = TakeOrder(robot, ds.Designator(customer_entity), orders=orders)
    sm.execute()

    rospy.loginfo("Orders {}".format(orders))
