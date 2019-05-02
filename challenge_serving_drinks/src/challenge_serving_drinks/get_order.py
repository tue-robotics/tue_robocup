# System
import random
import math

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states

from hmi import TimeoutException
from robocup_knowledge import knowledge_loader
from robot_skills.robot import Robot
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator
from robot_skills.util.kdl_conversions import frame_stamped, VectorStamped
from tue_msgs.msg import People
from Queue import Queue, Empty

# Knowledge
COMMON_KNOWLEDGE = knowledge_loader.load_knowledge("common")


class AskDrink(smach.State):
    """ Asks the operator what he/she would like to drink.

    This is based on the 'TakeOrder' class of the the restaurant challenge. Might
    be nice to merge these two.
    """
    def __init__(self, robot, drink_designator, max_tries=3, max_queries_per_try=3):
        # type (Robot, VariableDesignator) -> None
        """ Initialization method

        :param robot: robot api object
        :param drink_designator: designator in which the drink (entity type) is stored
        :param max_tries: (int) maximum number of times the robot asks which drink
        :param max_queries_per_try: (int) maximum number of queries to the HMI server per try
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._drink_designator = drink_designator
        self._max_tries = max_tries
        self._max_queries_per_try = max_queries_per_try

        # Speech grammars
        self._drinks_grammar, self._drinks_target = self._setup_drinks_grammar()

    @staticmethod
    def _setup_drinks_grammar():
        """ Sets up the grammar to ask which drink someone would like based on the objects in the knowlegde.

        :return: tuple(str, str) grammar and target
        """

        # Setup base
        grammar = 'O[P] -> BEVERAGE[P] | can i have a BEVERAGE[P] | i would like BEVERAGE[P] | can i get BEVERAGE[P]'
        grammar += ' | could i have BEVERAGE[P] | may i get BEVERAGE[P] | bring me BEVERAGE[P]'
        grammar += '\nBEVERAGE[B] -> BEV[B] | DET BEV[B]'
        grammar += '\nDET -> a | an'

        # Add drinks
        for d in COMMON_KNOWLEDGE.objects:
            if d["category"] == "drink":
                grammar += "\nBEV['{}'] -> {}[B]".format(d["name"], d["name"].replace('_', ' '))
        return grammar, "O"

    def execute(self, userdata=None):

        self._robot.head.look_at_standing_person()

        nr_tries = 0
        while nr_tries < self._max_tries and not rospy.is_shutdown():
            nr_tries += 1
            rospy.loginfo("AskDrink: attempt {} of {}".format(nr_tries, self._max_tries))

            # Ask the operator a question
            self._robot.speech.speak(
                random.choice([
                    "What would you like to drink?",
                    "Which drink would you like?",
                    "What would you like to quench your thirst?",
                    "What can I get you?",
                    "Please tell me which drink you want",
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
                "I understood that you would like {}, is that correct?".format(speech_result.semantics)
            )
            if not self._confirm():
                continue

            # Store the type in the designator
            self._drink_designator.write(str(speech_result.semantics))

            return "succeeded"

        self._robot.speech.speak("I am sorry but I cannot understand you. I will quit now", block=False)
        # ToDo: fill default?
        self._robot.head.cancel_goal()
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


class DetectWaving(smach.State):
    """ Waits for waving person.

    This is based on the 'WaitForCustomer' class of the the restaurant challenge. Might
    be nice to merge these two.
    """
    def __init__(self, robot, caller_id):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self._robot = robot
        self._caller_id = caller_id
        self._people_sub = rospy.Subscriber(robot.robot_name + '/persons', People, self.people_cb, queue_size=1)
        self.people_queue = Queue(maxsize=1)

    def people_cb(self, persons):
        if persons.people:
            rospy.logdebug('Received %d persons in the people cb', len(persons.people))
        self.people_queue.put(persons)

    def execute(self, userdata=None):

        self._robot.head.reset()
        self._robot.head.wait_for_motion_done()

        self._robot.speech.speak("Please wave if you want me to bring you something")

        head_samples = 20
        look_distance = 3.0
        look_angles = [0,
                       0,
                       0,
                       10,
                       -10,
                       20,
                       -20]
        self.clear_queue()

        waving_persons = []
        i = 0
        while not rospy.is_shutdown() and not waving_persons:
            header, waving_persons = self.wait_for_waving_person(head_samples=head_samples)

            angle = look_angles[i % len(look_angles)]
            rospy.loginfo('Still waiting... looking at %d degrees', angle)
            angle = math.radians(angle)
            head_goal = VectorStamped(x=look_distance * math.cos(angle),
                                      y=look_distance * math.sin(angle), z=1.3,
                                      frame_id="/%s/base_link" % self._robot.robot_name)
            self._robot.head.look_at_point(head_goal)
            i += 1

        if not waving_persons:
            return 'aborted'

        rospy.loginfo('waving persons: %s', waving_persons)
        if len(waving_persons) > 1:
            rospy.logwarn('using the first person')

        point = waving_persons[0].position
        pose = frame_stamped(header.frame_id, point.x, point.y, point.z)
        rospy.loginfo('update customer position to %s', pose)
        self._robot.ed.update_entity(id=self._caller_id, frame_stamped=pose, type="waypoint")

        return 'succeeded'

    def clear_queue(self):
        while True:
            try:
                self.people_queue.get_nowait()
                rospy.loginfo("trying")
            except Empty:
                # There is probably an old measurement blocking in the callback thread, also remove that one
                if not self.people_queue.empty():
                    self.people_queue.get()
                rospy.loginfo("returning")
                return

    def wait_for_waving_person(self, head_samples):
        waving_persons = []
        for i in range(0, head_samples):
            if rospy.is_shutdown():
                return

            people_received = self.wait_for_cb()
            rospy.logdebug('Got sample %d with seq %s', i, people_received.header.seq)
            for person in people_received.people:
                if {'RWave', 'LWave'}.intersection(set(person.tags)):
                    waving_persons.append(person)

            if waving_persons:
                break
        return people_received.header, waving_persons

    def wait_for_cb(self):
        timeout = 1

        people_received = People()
        while not rospy.is_shutdown() and not people_received.people:
            try:
                return self.people_queue.get(timeout=timeout)
            except Empty:
                rospy.logwarn('No people message received within %d seconds', timeout)


class GetOrder(smach.StateMachine):
    """ Gets an order. If succeeded, the person_designator and drink_designator are filled and can be used in subsequent
    states.

    """
    def __init__(self, robot, operator_name, drink_designator):
        # type: (Robot, str, VariableDesignator) -> None
        """ Initialization method

        :param robot: robot api object
        :param operator_name: name with which the operator will be stored in image recognition module
        :param drink_designator: (VariableDesignator) in which the drink to fetch is stored.
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:

            # For now:
            # * Say that everyone looks like having a drink
            # * Ask person to wave
            # * Drive to waiving person (a la restaurant)
            # * If not waving person (for fixed period):
            # * Ask person to step in front
            # * Wait (for fixed period/until operator is detected) for operator to step in front of the robot

            # Customer
            caller_id = "customer"
            caller_designator = states.util.designators.ed_designators.EdEntityDesignator(robot=robot, id=caller_id)

            smach.StateMachine.add(
                "ASK_FOR_WAVING",
                states.Say(
                    robot=robot,
                    sentence="Looks like everyone has a drink",
                    look_at_standing_person=True),
                transitions={"spoken": "ASK_STEP_IN_FRONT"}
            )

            smach.StateMachine.add(
                "WAIT_FOR_WAVING",
                DetectWaving(
                    robot=robot,
                    caller_id=caller_id),
                transitions={'succeeded': 'SAY_I_HAVE_SEEN',
                             'aborted': 'ASK_STEP_IN_FRONT'}
            )

            smach.StateMachine.add(
                'SAY_I_HAVE_SEEN',
                states.Say(
                    robot=robot,
                    sentence='I have seen a waving person, I will be there shortly!',
                    look_at_standing_person=True),
                transitions={"spoken": 'NAVIGATE_TO_WAVING'}
            )

            smach.StateMachine.add(
                'NAVIGATE_TO_WAVING',
                states.NavigateToObserve(
                    robot=robot,
                    entity_designator=caller_designator,
                    radius=1.1),
                transitions={'arrived': 'LEARN_OPERATOR',
                             'unreachable': 'ASK_STEP_IN_FRONT',
                             'goal_not_defined': 'WAIT_FOR_WAVING'}
            )

            smach.StateMachine.add(
                "ASK_STEP_IN_FRONT",
                states.Say(
                    robot=robot,
                    sentence="Please step in front of me to give your order",
                    look_at_standing_person=True),
                transitions={"spoken": "LEARN_OPERATOR"}
            )

            smach.StateMachine.add(
                "LEARN_OPERATOR",
                states.LearnPerson(
                    robot=robot,
                    person_name=operator_name,
                    nr_tries=5),
                transitions={"succeeded": "ASK_DRINK",
                             "failed": "failed"}  # ToDo: fallback
            )

            smach.StateMachine.add(
                "ASK_DRINK",
                AskDrink(
                    robot=robot,
                    drink_designator=drink_designator.writeable),
                transitions={"succeeded": "succeeded",
                             "failed": "failed"},
            )


if __name__ == "__main__":

    # Test code

    import sys

    rospy.init_node('get_drinks')

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        print("unknown robot")
        sys.exit()

    _robot = Robot()

    rospy.loginfo("Waiting for tf cache to be filled")
    rospy.sleep(0.5)  # wait for tf cache to be filled

    state = AskDrink(robot=_robot,
                     drink_designator=VariableDesignator(type=str).writeable,
                     max_tries=1,
                     max_queries_per_try=1,
                     )
    state.execute(None)
