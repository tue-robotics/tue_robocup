# System
import random

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds

from hmi import TimeoutException


class CheckAvailability(smach.State):
    """
    Hears the desired drink and checks its availability.
    """

    def __init__(self, robot, drink_designator, available_drinks_designator, unavailable_drink_designator,
                 objects, max_tries=6, max_queries_per_try=3):
        # type (Robot, VariableDesignator) -> None
        """
        Initialization method

        :param robot: robot api object
        :param drink_designator: (VariableDesignator) in which the drink to fetch is stored
        :param available_drinks_designator: (VariableDesignator) in which the available drinks are stored
        :param unavailable_drink_designator: (VariableDesignator) in which the unavailable drink is stored
        :param objects: Objects from common knowledge
        :param max_tries: (int) maximum number of times the robot asks which drink
        :param max_queries_per_try: (int) maximum number of queries to the HMI server per try
        """

        self._robot = robot
        self._drink_designator = drink_designator
        self._available_drinks_designator = available_drinks_designator
        self._unavailable_drink_designator = unavailable_drink_designator
        self._objects = objects
        self._max_tries = max_tries
        self._max_queries_per_try = max_queries_per_try
        self.trial = 0

        # Speech grammars
        self._drinks_grammar, self._drinks_target = self._setup_drinks_grammar()

        smach.State.__init__(self, outcomes=["available", "unavailable", "aborted", "bad_operator"])

    def _setup_drinks_grammar(self):
        """
        Sets up the grammar to ask which drink someone would like based on the objects in the knowlegde.
        :return: tuple(str, str) grammar and target
        """

        # Setup base
        grammar = 'O[P] -> BEVERAGE[P] | can i have a BEVERAGE[P] | i would like BEVERAGE[P] | can i get BEVERAGE[P]'
        grammar += ' | could i have BEVERAGE[P] | may i get BEVERAGE[P] | bring me BEVERAGE[P]'
        grammar += '\nBEVERAGE[B] -> BEV[B] | DET BEV[B]'
        grammar += '\nDET -> a | an'

        # Add drinks
        for d in self._objects:
            if d["category"] == "drink":
                grammar += "\nBEV['{}'] -> {}[B]".format(d["name"], d["name"].replace('_', ' '))
        return grammar, "O"

    def execute(self, userdata=None):

        self.trial += 1
        while self.trial < (self._max_tries + 1) and not rospy.is_shutdown():
            rospy.loginfo("AskDrink: attempt {} of {}".format(self.trial, self._max_tries))
            rospy.loginfo("Unavailable drink: {}".format(self._unavailable_drink_designator.resolve()))
            rospy.loginfo("Available drinks: {}".format(self._available_drinks_designator.resolve()))

            # Try to get the answer
            try:
                speech_result = self._query_drink(self._max_queries_per_try)
            except TimeoutException:
                return "aborted"

            # ToDo: Check if the drink is in the list of available drinks
            # Check if the drink is unavailable
            # if speech_result.semantics in self._available_drinks_designator.resolve():
            #     # Store the requested drink in the designator
            #     self._drink_designator.write(str(speech_result.semantics))
            #     rospy.loginfo("Requested drink: {}".format(self._drink_designator.resolve()))
            #     return "available"

            if speech_result.semantics == self._unavailable_drink_designator.resolve():
                return "unavailable"
            else:
                # Store the requested drink in the designator
                self._drink_designator.write(str(speech_result.semantics))
                rospy.loginfo("Requested drink: {}".format(self._drink_designator.resolve()))
                return "available"

        return "bad_operator"

    def _query_drink(self, max_tries):
        """
        Queries the HMI server for <max_tries> times until a result has been obtained
        :return: (HMIResult) (N.B.: the 'semantics' attribute should be a dict with 'beverage' key)
        :raises: (TimeoutException) if robot didn't hear anything
        """
        attempt = 0
        while not rospy.is_shutdown():
            attempt += 1
            try:
                speech_result = self._robot.hmi.query(description="Can I please take your order",
                                                      grammar=self._drinks_grammar,
                                                      target=self._drinks_target)
                return speech_result
            except TimeoutException:
                if attempt >= max_tries:
                    raise  # Re-raise

            # We only end up here if the exception was caught
            self._robot.speech.speak(random.choice([
                "I'm sorry, can you repeat",
                "Please repeat your order, I didn't hear you",
                "I didn't get your order, can you repeat it",
                "Please speak up, as I didn't hear your order",
            ]))


class AskDrink(smach.StateMachine):
    """
    Asks the operator what he/she would like to drink.
    """
    def __init__(self, robot, operator_name, drink_designator, available_drinks_designator,
                 unavailable_drink_designator, objects):
        # type (Robot, VariableDesignator) -> None
        """
        Initialization method

        :param robot: robot api object
        :param operator_name: (EntityDesignator) in which the operator's name is stored
        :param drink_designator: (VariableDesignator) in which the drink to fetch is stored
        :param available_drinks_designator: (VariableDesignator) in which the available drinks are stored
        :param unavailable_drink_designator: (VariableDesignator) in which the unavailable drink is stored
        :param objects: Objects from common knowledge
        """

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:

            # Ask for order
            smach.StateMachine.add("RISE_FOR_HMI",
                                   states.RiseForHMI(robot=robot),
                                   transitions={"succeeded": "ASK_FOR_ORDER",
                                                "failed": "ASK_FOR_ORDER"})

            smach.StateMachine.add("ASK_FOR_ORDER",
                                   states.Say(robot=robot,
                                              sentence=random.choice(["What would you like to drink?",
                                                                      "Which drink would you like?",
                                                                      "What would you like to quench your thirst?",
                                                                      "What can I get you?",
                                                                      "Please tell me which drink you want"]),
                                              look_at_standing_person=True),
                                   transitions={"spoken": "CHECK_AVAILABILITY"})

            # Check availability
            smach.StateMachine.add("CHECK_AVAILABILITY",
                                   CheckAvailability(robot=robot, drink_designator=drink_designator,
                                                     available_drinks_designator=available_drinks_designator,
                                                     unavailable_drink_designator=unavailable_drink_designator,
                                                     objects=objects),
                                   transitions={"available": "ASK_FOR_CONFIRMATION",
                                                "unavailable": "STATE_UNAVAILABLE",
                                                "aborted": "aborted",
                                                "bad_operator": "SAY_BAD_OPERATOR"})


            # Ask for confirmation
            smach.StateMachine.add("ASK_FOR_CONFIRMATION",
                                   states.Say(robot=robot,
                                              sentence=DescriptionStrDesignator("confirmation_available",
                                                                                drink_designator, operator_name),
                                              look_at_standing_person=True),
                                   transitions={"spoken": "HEAR_CONFIRMATION"})


            # Hear the confirmation
            smach.StateMachine.add("HEAR_CONFIRMATION",
                                   states.HearOptions(robot=robot, options=["yes", "no"]),
                                   transitions={"yes": "succeeded",
                                                "no": "ASK_FOR_ORDER",
                                                "no_result": "ASK_FOR_ORDER"})  # ToDo: fallback?

            # Announce that the drink is unavailable
            smach.StateMachine.add("STATE_UNAVAILABLE",
                                   states.Say(robot=robot,
                                              sentence=DescriptionStrDesignator("state_unavailable",
                                                                                unavailable_drink_designator,
                                                                                operator_name),
                                              look_at_standing_person=True),
                                   transitions={"spoken": "ASK_FOR_ORDER"})

            # Tell the operator to stop fucking around!
            smach.StateMachine.add("SAY_BAD_OPERATOR",
                                   states.Say(robot=robot,
                                              sentence="I'm not going to ask you again as I have already informed you multiple times that your request is unavailable",
                                              look_at_standing_person=True),
                                   transitions={"spoken": "aborted"})


class AskAvailability(smach.State):
    """ Asks the bartender which is the unavailable drink.

    This is based on the 'TakeOrder' class of the the restaurant challenge. Might
    be nice to merge these two.
    """
    def __init__(self, robot, unavailable_drink_designator, objects, max_tries=3, max_queries_per_try=3):
        # type (Robot, VariableDesignator) -> None
        """ Initialization method

        :param robot: robot api object
        :param unavailable_drink_designator: (VariableDesignator) in which the unavailable drink is stored
        :param objects: Objects from common knowledge
        :param max_tries: (int) maximum number of times the robot asks which drink
        :param max_queries_per_try: (int) maximum number of queries to the HMI server per try
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._unavailable_drink_designator = unavailable_drink_designator
        self._objects = objects
        self._max_tries = max_tries
        self._max_queries_per_try = max_queries_per_try
        self._trial = 0

        # Speech grammars
        self._drinks_grammar, self._drinks_target = self._setup_drinks_grammar()

    def _setup_drinks_grammar(self):
        """ Sets up the grammar to ask which drink is unavailable based on the objects in the knowlegde.

        :return: tuple(str, str) grammar and target
        """

        # Setup base
        grammar = 'O[P] -> BEVERAGE[P] | DET unavailable drink is BEVERAGE[P] | BEVERAGE[P] is unavailable'
        grammar += '\nBEVERAGE[B] -> BEV[B] | DET BEV[B]'
        grammar += '\nDET -> a | an | the'

        # Add drinks
        for d in self._objects:
            if d["category"] == "drink":
                grammar += "\nBEV['{}'] -> {}[B]".format(d["name"], d["name"].replace('_', ' '))
        return grammar, "O"

    def execute(self, userdata=None):

        self._robot.head.look_at_standing_person()

        while self._trial < self._max_tries and not rospy.is_shutdown():
            self._trial += 1
            rospy.loginfo("AskAvailable: attempt {} of {}".format(self._trial, self._max_tries))

            # Ask the bartender a question
            self._robot.speech.speak(
                random.choice([
                    "I am unable to inspect the bar. Would you tell me which is the unavailable drink?",
                    "Would you tell me which is the unavailable drink?",
                    "Please tell me which drink is unavailable",
                ]),
                block=True
            )

            # Try to get the answer
            try:
                speech_result = self._query_drink(self._max_queries_per_try)
            except TimeoutException:
                continue

            # Ask for confirmation
            self._robot.speech.speak(
                "I understood that the {} is unavailable, is that correct?".format(speech_result.semantics)
            )
            if not self._confirm():
                continue

            # Store the type in the designator
            self._unavailable_drink_designator.write(str(speech_result.semantics))
            rospy.loginfo("Unavailable drink: {}".format(self._unavailable_drink_designator.resolve()))

            return "succeeded"

        self._robot.speech.speak("I am sorry but I cannot understand you. I will continue with my tasks", block=False)
        self._robot.head.cancel_goal()
        rospy.loginfo("Unavailable drink still unknown")
        return "failed"

    def _query_drink(self, max_tries):
        """
        Queries the HMI server for <max_tries> times until a result has been obtained
        :return: (HMIResult) (N.B.: the 'semantics' attribute should be a dict with 'beverage' key)
        :raises: (TimeoutException) if robot didn't hear anything
        """
        attempt = 0
        while not rospy.is_shutdown():
            attempt += 1
            try:
                speech_result = self._robot.hmi.query(description="Which is the unavailable beverage?",
                                                      grammar=self._drinks_grammar,
                                                      target=self._drinks_target)
                return speech_result
            except TimeoutException:
                if attempt >= max_tries:
                    raise  # Re-raise

            # We only end up here if the exception was caught
            self._robot.speech.speak(random.choice([
                "I'm sorry, can you repeat",
                "Please repeat the drink, I didn't hear you",
                "I didn't get the drink, can you repeat it",
                "Please speak up, as I didn't hear what you just said",
            ]))

    def _confirm(self):
        """
        Queries the HMI server for confirmation
        :return: (bool) whether the understood beverage was correct
        """
        try:
            speech_result = self._robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;"
                                                                                          "T[False] -> no", target="T")
        except TimeoutException:
            return False

        return speech_result.semantics


class DescriptionStrDesignator(ds.Designator):
    def __init__(self, message_type, drink_request_des, operator_name_des, name=None):
        super(DescriptionStrDesignator, self).__init__(resolve_type=str, name=name)

        ds.check_type(operator_name_des, str)
        ds.check_type(drink_request_des, str)

        self.message_type = message_type
        self.operator_name_des = operator_name_des
        self.drink_request_des = drink_request_des

    def _resolve(self):
        operator_name = self.operator_name_des.resolve()
        if not operator_name:
            rospy.logerr("Could not resolve operator name")
        drink_request = self.drink_request_des.resolve()
        if not drink_request:
            rospy.logerr("Could not resolve drink request")
        if self.message_type == "found_operator":
            return "Hey {name}, I'm bringing your {drink}".format(name=operator_name, drink=drink_request)
        elif self.message_type == "not_found_operator":
            return "Hey {name} I'm back."\
                   "Please come to me to receive your {drink}".format(name=operator_name, drink=drink_request)
        elif self.message_type == "fallback_bar":
            return "Oh, I cannot inspect the bar. Please hand me over the {drink}".format(drink=drink_request)
        elif self.message_type == "confirmation_available":
            return "I understood that you would like {drink}, is that correct?".format(drink=drink_request)
        elif self.message_type == "state_unavailable":
            return random.choice([
                    "Unfortunately we don't have {drink}".format(drink=drink_request),
                    "The requested {drink} is unavailable".format(drink=drink_request),
                    "I'm sorry but {drink} is out of stock".format(drink=drink_request),
            ])
        else:
            rospy.logerr("No correct message type defined for DescriptionStrDesignator")


if __name__ == "__main__":

    # Test code

    rospy.init_node('get_drinks')

    from robot_skills.get_robot import get_robot_from_argv

    _robot = get_robot_from_argv(index=1)

    rospy.loginfo("Waiting for tf cache to be filled")
    rospy.sleep(0.5)  # wait for tf cache to be filled

    state = AskDrink(robot=_robot,
            drink_designator=ds.VariableDesignator(resolve_type=str).writeable)
    state.execute(None)
