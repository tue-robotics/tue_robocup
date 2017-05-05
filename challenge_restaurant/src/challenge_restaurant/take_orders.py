#!/usr/bin/python

# System
import random

# ROS
import rospy
import smach

# TU/e Robotics
from hmi import TimeoutException
from robocup_knowledge import knowledge_loader

# Knowledge
knowledge = knowledge_loader.load_knowledge("challenge_restaurant")


class TakeOrder(smach.State):
    """ Take an order """

    def __init__(self, robot, location, orders):
        """ Constructor

        :param robot: robot object
        :param location: string indicating the location
        :param orders: Python dict in which the orders will be stored
        :return:
        succeeded: understood correctly
        failed: didn't hear anything or exceeded maximum number of tries
        misunderstood: misunderstood, might try again
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'misunderstood'])

        self._robot = robot
        self._location = location
        self._orders = orders
        self._nr_tries = 0
        self._max_tries = 5

    def _confirm(self):
        cgrammar = """
        C[P] -> A[P]
        A['yes'] -> yes
        A['no'] -> no
        """
        try:
            speech_result = self._robot.hmi.query(description="Is this correct?",
                                                 grammar=cgrammar, target="C")
        except TimeoutException:
            return False
        return speech_result.semantics == "yes"

    def execute(self, userdata):
        self._nr_tries += 1
        self._robot.head.look_at_ground_in_front_of_robot(3)
        self._robot.speech.speak("Which combo or beverage do you want?")

        order = None
        while not order:

            # #### TEMP: before speech recognition is there
            # self._robot.speech.speak("I'm sorry, but I'm not able to understand your order")
            #
            # self._orders["beverage"] = {"location": "table1", "name": "beer"}
            # self._orders["combo"] = {"location": "table2", "name": "nuts and bolts"}
            #
            # return "succeeded"
            #####

            self._robot.speech.speak("Hello there, can I please take your order")
            count = 0
            while not rospy.is_shutdown():
                count += 1

                try:
                    speech_result = self._robot.hmi.query(description="Can I please take your order",
                                                         grammar=knowledge.order_grammar, target="O")
                    break
                except TimeoutException:
                    if count < 5:
                        self._robot.speech.speak(random.choice(["I'm sorry, can you repeat",
                                                               "Please repeat your order, I didn't hear you",
                                                               "I didn't get your order, can you repeat it",
                                                               "Please speak up, as I didn't hear your order"]))
                    else:
                        self._robot.speech.speak("I am sorry but I cannot understand you. I will quit now", block=False)
                        self._robot.head.cancel_goal()
                        return "failed"

            # Now: confirm
            if "beverage" in speech_result.semantics:
                self._robot.speech.speak("I understood that you would like {}, "
                                         "is this correct?".format(speech_result.semantics['beverage']))
            elif "food1" and "food2" in speech_result.semantics:
                self._robot.speech.speak("I understood that you would like {} and {}, "
                                         "is this correct?".format(speech_result.semantics['food1'],
                                                                   speech_result.semantics['food2']))

            if self._confirm():
                self._order = speech_result.semantics
                self._robot.head.cancel_goal()
                self._robot.speech.speak("Ok, I will get your order", block=False)
                return "succeeded"
            else:
                if self._nr_tries < self._max_tries:
                    self._robot.head.cancel_goal()
                    return "misunderstood"
                else:
                    self._robot.speech.speak("I am sorry but I cannot understand you. I will quit now", block=False)
                    self._robot.head.cancel_goal()
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

    def execute(self, userdata):
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("Mr. Barman I have some orders.")

        sentence_combo = ""
        sentence_beverage = ""
        sentence_final = ""

        if "beverage" in self._orders:
            sentence_beverage = "Table %s would like to have the beverage %s" % (self._orders["beverage"]["location"],
                                                                                 self._orders["beverage"]["name"])
            sentence_final = sentence_beverage
        if "combo" in self._orders:
            sentence_combo = "Table %s wants the combo %s" % (self._orders["combo"]["location"],
                                                              self._orders["combo"]["name"])
            sentence_final = sentence_combo

        ''' add a COMA and an AND abetween sentences better understanding '''
        if "combo" in self._orders and "beverage" in self._orders:
            sentence_final = sentence_beverage + ", and " + sentence_combo

        self._robot.speech.speak(sentence_final)

        self._robot.head.cancel_goal()

        return "spoken"
