#!/usr/bin/python

# ROS
import smach


class TakeOrder(smach.State):
    """ Take an order """

    def __init__(self, robot, location, orders):
        """ Constructor

        :param robot: robot object
        :param location: string indicating the location
        :param orders: Python dict in which the orders will be stored
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot
        self._location = location
        self._orders = orders

    def _confirm(self, tries=3):
        for i in range(0, tries):
            result = self._robot.ears.recognize("<option>", {"option": ["yes", "no"]})
            if result and result.result:
                answer = result.result
                return answer == "yes"

            if i != tries - 1:
                self._robot.speech.speak("Please say yes or no")
        return False

    def execute(self, userdata):
        self._robot.head.look_at_ground_in_front_of_robot(3)
        self._robot.speech.speak("Which combo or beverage do you want?")

        order = None
        while not order:

            # #### TEMP: before speech recognition is there
            self._robot.speech.speak("I'm sorry, but I'm not able to understand your order")

            self._orders["beverage"] = {"location": "table1", "name": "beer"}
            self._orders["combo"] = {"location": "table2", "name": "nuts and bolts"}

            return "succeeded"
            #####

            result = self._robot.ears.recognize(knowledge.order_spec, knowledge.order_choices)
            if result and result.result:

                self._robot.speech.speak("I heard %s, is this correct?" % result.result)
                if not self._confirm():
                    continue

                if "beverage" in result.choices:
                    order = result.choices["beverage"]
                    self._orders["beverage"] = {"location": self._location, "name": order}
                elif "food1" and "food2" in result.choices:
                    if result.choices["food1"] == result.choices["food2"]:
                        self._robot.speech.speak("Sorry, I dit not understand, please repeat.")
                    else:
                        order = "%s and %s" % (result.choices["food1"], result.choices["food2"])
                        self._orders["combo"] = {"location": self._location, "name": order}
            else:
                self._robot.speech.speak("Sorry, I dit not understand, please repeat.")

        self._robot.head.cancel_goal()

        self._robot.speech.speak("Ok, I will get you %s" % order, block=False)

        print "\n\n Current orders: \n\n"
        print self._orders

        if "combo" in self._orders and "beverage" in self._orders:
            return "succeeded"
        else:
            return "succeeded"


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
