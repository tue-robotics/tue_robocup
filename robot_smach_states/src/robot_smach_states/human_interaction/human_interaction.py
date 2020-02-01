# System
import math
import random
import time
from string import Formatter

# ROS
import rospy
import smach

# TU/e Robotics
from hmi import TimeoutException
import robot_smach_states.util.designators as ds
from hmi import HMIResult

# Say: Immediate Say with optional named placeholders for designators
# Hear: Immediate hear
# Ask: Interaction, say + hear
from robot_skills.robot import Robot


class Say(smach.State):
    """
    Say a sentence or pick a random one from a list, which then is formatted with designators which are resolved on
    runtime. The main sentence can be a str, [str] or a Designator to str or [str]

    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>>
    >>> say = Say(robot, ["a", "b", "c"])
    >>> say.execute()
    'spoken'
    >>>
    >>> say1 = Say(robot, ["Hey {a}", "He {a}", "Hoi {a}"], a=ds.VariableDesignator("hero"))
    >>> #Repeat command 50 times, every time it should succeed and return "spoken"
    >>> outcomes1 = [say.execute() for i in range(50)]
    >>> assert all(outcome == "spoken" for outcome in outcomes1)
    >>>
    >>> say2 = Say(robot, ds.VariableDesignator('aap'))
    >>> say2.execute()
    'spoken'
    >>> robot.speech.speak.assert_called_with('aap', None, None, None, None, True)
    >>>
    >>> des = ds.VariableDesignator(["Hey {a}", "He {a}", "Hoi {a}"], resolve_type=[str])
    >>> say3 = Say(robot, des, a=ds.VariableDesignator("hero"))
    >>> #Repeat command 50 times, every time it should succeed and return "spoken"
    >>> outcomes3 = [say3.execute() for i in range(50)]
    >>> assert all(outcome == "spoken" for outcome in outcomes3)
    >>>
    >>> des2 = ds.VariableDesignator(["Hey", "He", "Hoi"])
    >>> say4 = Say(robot, des2, a=ds.VariableDesignator("hero"))
    >>> #Repeat command 50 times, every time it should succeed and return "spoken"
    >>> outcomes4 = [say4.execute() for i in range(50)]
    >>> assert all(outcome == "spoken" for outcome in outcomes4)
    >>>
    >>> say5 = Say(robot, des2)
    >>> #Repeat command 50 times, every time it should succeed and return "spoken"
    >>> outcomes5 = [say5.execute() for i in range(50)]
    >>> assert all(outcome == "spoken" for outcome in outcomes5)
    >>>
    >>> say6 = Say(robot, des, b=ds.VariableDesignator("hero"))
    >>> say6.execute()  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    RuntimeError: ...
    >>>
    >>> des3 = ds.VariableDesignator(resolve_type=str).writeable
    >>> des3.write('banana')
    >>> say6 = Say(robot, des3)
    >>> say6.execute()
    'spoken'
    >>> robot.speech.speak.assert_called_with('banana', None, None, None, None, True)
    """

    def __init__(self, robot, sentence=None, language=None, personality=None, voice=None, mood=None, block=True,
                 look_at_standing_person=False, **place_holders):
        """
        Constructor

        State exits with 'spoken'.

        :param robot: robot object
        :type robot: Robot
        :param sentence: Sentence to be spoken, can contain place holders to  be filled in at runtime
        :type sentence: str, [str], designator to str or [str]
        :param language: Language of speech module
        :type language: str
        :param personality: Personality of speech module
        :type personality: str
        :param voice: Voice of speech module
        :type voice: str
        :param mood: Mood of speech module
        :type mood: str
        :param block: Wait for talking to be completed before returning, if true
        :type block: bool
        :param look_at_standing_person: Look at standing person if true, otherwise keep current head pose
        :type look_at_standing_person: bool
        :param place_holders: place holders to be filled in at runtime
        :type place_holders: designator to str
        """
        smach.State.__init__(self, outcomes=["spoken"])

        ds.check_type(sentence, [str], str)
        assert(isinstance(language, str) or isinstance(language, type(None)))
        assert(isinstance(personality, str) or isinstance(personality, type(None)))
        assert(isinstance(voice, str) or isinstance(voice, type(None)))
        assert(isinstance(mood, str) or isinstance(mood, type(None)))
        assert(isinstance(block, bool))

        assert(all(isinstance(v, ds.Designator) for v in place_holders.values()))

        self.ph_designators = place_holders

        if isinstance(sentence, str) or isinstance(sentence, list):
            self._check_place_holders(sentence)

        self.robot = robot
        self.sentence = sentence
        self.language = language
        self.personality = personality
        self.voice = voice
        self.mood = mood
        self.block = block
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata=None):
        if not self.sentence:
            rospy.logerr("sentence = None, not saying anything...")
            return "spoken"

        if hasattr(self.sentence, "resolve"):
            sentence = self.sentence.resolve()
            self._check_place_holders(sentence)
        else:
            sentence = self.sentence

        if not isinstance(sentence, str) and isinstance(sentence, list):
            sentence = random.choice(sentence)

        resolved_ph = {k: v.resolve() for k, v in self.ph_designators.items()}
        sentence = sentence.format(**resolved_ph)

        if self.look_at_standing_person:
            self.robot.head.look_at_standing_person()
        self.robot.speech.speak(str(sentence), self.language, self.personality, self.voice, self.mood, self.block)

        return "spoken"

    def _check_place_holders(self, sentence):
        if isinstance(sentence, list):
            for sen in sentence:
                self._check_place_holders(sen)
            return

        place_holders = set(x[1] for x in Formatter().parse(sentence) if x[1] is not None)
        missing_ph = place_holders - set(self.ph_designators.keys())
        if missing_ph:
            raise RuntimeError("Not all named place holders are provided, missing: {}".
                               format(", ".join(map(str, missing_ph))))


class HearOptions(smach.State):
    """
    Hear one of the options
    """
    def __init__(self, robot, options, timeout=10, look_at_standing_person=True):
        # type:(Robot, list, (float, int), bool) -> None
        """

        :param robot: (Robot) robot api object
        :param options: List of strings with the options the robot can hear
        :param timeout: (float, int) indicating when the robot has to timeout
        :param look_at_standing_person: bool indicating whether the robot should look at the person giving the command
        """
        outcomes = list(options)  # make a copy
        outcomes.append("no_result")
        smach.State.__init__(self, outcomes=outcomes)
        self._options = options
        self._robot = robot
        assert(isinstance(timeout, (float, int)))
        self._timeout = timeout
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata=None):
        if self.look_at_standing_person:
            self._robot.head.look_at_standing_person()

        try:
            answer = self._robot.hmi.query('Which option?', 'T -> ' + ' | '.join(self._options), 'T',
                                           timeout=self._timeout)
        except TimeoutException:
            return 'no_result'

        if self.look_at_standing_person:
            self._robot.head.cancel_goal()

        if answer.sentence in self._options:
            return answer.sentence

        return 'no_result'


class HearOptionsExtra(smach.State):
    """
    Listen to what the user said, based on a pre-constructed sentence

    Keyword arguments:
    spec_designator -- sentence that is supposed to be heard
    choices_designator -- list of choices for words in the sentence
    speech_result_designator -- variable where the result is stored
    time_out -- timeout in case nothing is heard

    Example of usage:
        from hmi import HMIResult
        spec = ds.Designator("T --> <name>)|<name>)")
        choices = ds.Designator({"name"  : names_list,
                              "prefix": ["My name is", "I'm called"]})
        answer = ds.VariableDesignator(resolve_type=HMIResult)
        state = HearOptionsExtra(self.robot, spec, choices, answer.writeable)
        outcome = state.execute()

        if outcome == "heard":
            name = answer.resolve().choices["name"]

    >>> from robot_skills.mockbot import Mockbot
    >>> from hmi import HMIResult
    >>> mockbot = Mockbot()
    >>> import robot_smach_states.util.designators as ds
    >>> import os
    >>> spec = "T[O] -> OPTIONS[O]"+os.linesep  # Newlines are needed, doctests doesn't accept 'slash-n"
    >>> spec += "OPTIONS['foo'] -> foo"+os.linesep
    >>> spec += "OPTIONS['bar'] -> bar"+os.linesep
    >>> spec = ds.Designator(spec)
    >>> answer = ds.VariableDesignator(resolve_type=HMIResult)
    >>> state = HearOptionsExtra(mockbot, spec, answer.writeable)
    >>> outcome = state.execute()
    """

    def __init__(self, robot,
                 spec_designator,
                 speech_result_designator,
                 timeout=10,
                 look_at_standing_person=True):
        smach.State.__init__(self, outcomes=["heard", "no_result"])

        self.robot = robot

        ds.check_resolve_type(spec_designator, str)
        ds.check_resolve_type(speech_result_designator, HMIResult)
        ds.is_writeable(speech_result_designator)

        self.spec_designator = spec_designator
        self.speech_result_designator = speech_result_designator
        self.timeout = timeout
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata=None):
        spec = self.spec_designator.resolve()

        if not spec:
            rospy.logerr("Could not resolve spec")
            return "no_result"

        if self.look_at_standing_person:
            self.robot.head.look_at_standing_person()

        try:
            answer = self.robot.hmi.query('Which option?', spec, 'T',  # TODO: T needs to also be configurable
                                          timeout=self.timeout)

            if self.look_at_standing_person:
                self.robot.head.cancel_goal()

            if answer:
                if answer.semantics:
                    self.speech_result_designator.write(answer)
                    return "heard"
            else:
                self.robot.speech.speak("Something is wrong with my ears, please take a look!")
        except TimeoutException:
            return 'no_result'

        return "no_result"


class AskContinue(smach.StateMachine):
    def __init__(self, robot, timeout=10):
        smach.StateMachine.__init__(self, outcomes=['continue', 'no_response'])

        with self:
            smach.StateMachine.add('SAY',
                                   Say(robot,
                                       random.choice(["I will continue my task if you say continue.",
                                                      "Please say continue so that I can continue my task.",
                                                      "I will wait until you say continue."])),
                                   transitions={'spoken': 'HEAR'})

            smach.StateMachine.add('HEAR',
                                   HearOptions(robot, ['continue'], timeout),
                                   transitions={'continue': 'continue',
                                                'no_result': 'no_response'})


class AskYesNo(HearOptions):
    def __init__(self, robot, timeout=10):
        HearOptions.__init__(self, robot, ['yes', 'no'], timeout)


class WaitForPersonInFront(smach.State):
    def __init__(self, robot, attempts=1, sleep_interval=1.0):
        """
        Waits for a person to be found in front of the robot. Attempts to wait a number of times with a sleep interval

        :param robot: (Robot) robot api object
        :param attempts: (int) number of attempts the robot will take
        :param sleep_interval: (float) time the robot waits between checking for an operator
        """
        smach.State.__init__(self, outcomes=["success", "failed"])
        self.robot = robot
        self.attempts = attempts
        self.sleep_interval = sleep_interval

    def execute(self, userdata=None):
        self.robot.head.look_at_standing_person()

        for i in range(self.attempts):
            image_data = self.robot.perception.get_rgb_depth_caminfo()
            success, found_people_ids = self.robot.ed.detect_people(*image_data)
            if any(found_people_ids):
                rospy.loginfo("There are {} people in front of me (1 is enough): {}".format(
                    len(found_people_ids),
                    found_people_ids)
                )
                return 'success'
            rospy.sleep(self.sleep_interval)
        return 'failed'


class LearnPerson(smach.State):
    """ Smach state to learn a person

    """

    def __init__(self, robot, person_name="", name_designator=None, nr_tries=5):
        """ Constructor

        :param robot: robot object
        :param person_name: string indicating the name that will be given
        :param name_designator: designator returning a string with the name of the person. This will be used if no
            person name is provided
        :param nr_tries: maximum number of tries
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._robot = robot
        self._person_name = person_name
        if name_designator:
            ds.check_resolve_type(name_designator, str)
        self._name_designator = name_designator
        self._nr_tries = nr_tries

    def execute(self, userdata=None):

        # Look up
        self._robot.speech.speak("Please look at me while I admire your beauty", block=False)
        self._robot.head.look_at_standing_person()
        self._robot.head.wait_for_motion_done()

        # if person_name is empty then try to get it from designator
        if not self._person_name:
            person_name = self._name_designator.resolve()
            rospy.loginfo("[LearnPerson] Resolved name {}".format(person_name))

            # if there is still no name, quit the learning
            if not person_name:
                rospy.logerr("[LearnPerson] No name was provided. Quitting the learning")
                self._robot.head.reset()
                return "failed"
        else:
            person_name = self._person_name

        # Learn the face (try for a maximum of nr_tries times
        for i in range(self._nr_tries):
            if self._robot.perception.learn_person(name=person_name):
                self._robot.head.reset()
                return "succeeded"

        # If we end up here, learning failed
        self._robot.speech.speak("There's something in my eyes, I might not be able to remember you")
        self._robot.head.reset()
        return "failed"


def learn_person_procedure(robot, person_name="", n_samples=5, timeout=5.0):
    """
    Starts the learning process that will save n_samples of the closest person's face.
    It ends when the number of snapshots is reached or when a timeout occurs

    Returns: number of samples saved. If smaller than what was requested, then a timeout occured
    """

    # if there is no name, quit the learning
    if not person_name:
        rospy.logwarn("No name was provided. Quitting the learning!")
        return

    count = 0
    start_time = time.time()
    while count < n_samples:
        rospy.logerr("robot.ed.learn_ person(person_name), dissapeared! I will return false")
        if False:
            if count == 0:
                robot.speech.speak("Hi there!")

            # reset timer
            start_time = time.time()

            count += 1

            if count == math.ceil(n_samples / 2):
                robot.speech.speak("Almost done, keep looking.", block=False)
        else:
            rospy.loginfo ("[LearnPersonProcedure] " + "No person found.")
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                rospy.loginfo ("[LearnPersonProcedure] " + "Learn procedure timed out!")
                return count

        rospy.loginfo("[LearnPersonProcedure] " + "Completed {0}/{1}".format(count, n_samples))

    rospy.loginfo("[LearnPersonProcedure] " + "Learn procedure completed!")

    return count


class AskPersonName(smach.State):
    """
    Ask the person's name, and try to hear one of the given names
    """

    def __init__(self, robot, person_name_des, name_options, default_name='Operator', nr_tries=2):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'timeout'])

        self.robot = robot
        self.person_name_des = person_name_des
        self.default_name = default_name
        self.name_options = name_options
        self._nr_tries = nr_tries

    def execute(self, userdata=None):
        limit_reached = 0

        for i in range(self._nr_tries):
            rospy.loginfo("AskPersonName")

            self.robot.speech.speak("What is your name?", block=True)

            names_spec = "T['name': N] -> NAME[N]\n\n"
            for dn in self.name_options:
                names_spec += "NAME['{name}'] -> {name}\n".format(name=dn)
            spec = ds.Designator(names_spec)

            answer = ds.VariableDesignator(resolve_type=HMIResult)
            state = HearOptionsExtra(self.robot, spec, answer.writeable)
            try:
                outcome = state.execute()

                if not outcome == "heard":
                    limit_reached += 1
                    if limit_reached == self._nr_tries:
                        self.person_name_des.write(self.default_name)

                        rospy.logwarn(
                            "Speech recognition outcome was not successful. Using default name '{}'".format(
                                self.person_name_des.resolve()))
                        return 'failed'

                if outcome == "heard":
                    try:
                        rospy.logdebug("Answer: '{}'".format(answer.resolve()))
                        name = answer.resolve().semantics["name"]
                        rospy.loginfo("This person's name is: '{}'".format(name))
                        self.person_name_des.write(str(name))

                        rospy.loginfo("Result received from speech recognition is '" + name + "'")
                    except KeyError as ke:
                        rospy.loginfo("KeyError resolving the name heard: " + str(ke))
                        return 'failed'
                    return 'succeeded'
            except TimeoutException:
                return "timeout"

        return 'succeeded'


if __name__ == "__main__":
    rospy.init_node('human_interaction_doctest')  # Needed to instantiate a Robot subclass
    import doctest

    doctest.testmod()
