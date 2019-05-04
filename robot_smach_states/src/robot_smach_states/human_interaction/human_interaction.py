#! /usr/bin/env python

# System
import math
import random
import time

# ROS
import rospy
import smach

# TU/e Robotics
from hmi import TimeoutException
import robot_smach_states.util.designators as ds
from robot_smach_states.utility import WaitForDesignator
from hmi import HMIResult


# Say: Immediate say
# Hear: Immediate hear
# Ask: Interaction, say + hear

##########################################################################################################################################


class Say(smach.State):
    """Say a sentence or pick a random one from a list.

    >>> from mock import MagicMock
    >>> robot = MagicMock()
    >>> robot.speech = MagicMock()
    >>> robot.speech.speak = MagicMock()
    >>>
    >>> sf = Say(robot, ["a", "b", "c"])
    >>> #Repeat command 50 times, every time it should succeed and return "spoken"
    >>> outcomes = [sf.execute() for i in range(50)]
    >>> assert all(outcome == "spoken" for outcome in outcomes)
    >>>
    >>> #After many calls, all options in the list will very likely have been called at least one.
    >>> #robot.speech.speak.assert_any_call('a', 'us', 'kyle', 'default', 'excited', True)
    >>> #robot.speech.speak.assert_any_call('b', 'us', 'kyle', 'default', 'excited', True)
    >>> #robot.speech.speak.assert_any_call('c', 'us', 'kyle', 'default', 'excited', True)"""

    def __init__(self, robot, sentence=None, language=None, personality=None, voice=None, mood=None, block=True,
                 look_at_standing_person=False):
        smach.State.__init__(self, outcomes=["spoken"])
        ds.check_type(sentence, str, list)
        # ds.check_type(language, str)
        # ds.check_type(personality, str)
        # ds.check_type(voice, str)
        # ds.check_type(mood, str)
        ds.check_type(block, bool)

        self.robot = robot
        self.sentence = sentence
        self.language = language
        self.personality = personality
        self.voice = voice
        self.mood = mood
        self.block = block
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata=None):
        # robot.head.look_at_standing_person()

        if not self.sentence:
            rospy.logerr("sentence = None, not saying anything...")
            return "spoken"

        if not isinstance(self.sentence, str) and isinstance(self.sentence, list):
            self.sentence = random.choice(self.sentence)

        sentence = str(self.sentence.resolve() if hasattr(self.sentence, "resolve") else self.sentence)

        if self.look_at_standing_person:
            self.robot.head.look_at_standing_person()
        self.robot.speech.speak(sentence, self.language, self.personality, self.voice, self.mood, self.block)

        # robot.head.cancel_goal()
        # ToDo: hack
        rospy.sleep(len(sentence)*0.05)

        return "spoken"


class HearOptions(smach.State):
    """Hear one of the options
    """

    def __init__(self, robot, options, timeout=rospy.Duration(10), look_at_standing_person=True):
        outcomes = list(options)  # make a copy
        outcomes.append("no_result")
        smach.State.__init__(self, outcomes=outcomes)
        self._options = options
        self._robot = robot
        self._timeout = timeout
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata=None):
        if self.look_at_standing_person:
            self._robot.head.look_at_standing_person()

        try:
            answer = self._robot.hmi.query('Which option?', 'T -> ' + ' | '.join(self._options), 'T',
                                           timeout=self._timeout.to_sec())
        except TimeoutException:
           # self._robot.speech.speak("Something is wrong with my ears, please take a look!")
            return 'no_result'
        # except Exception as e:
        #     rospy.logfatal(
        #         e.message)  # This should be a temp addition. If this exception is thrown that means that there is a bug to be fixed
        #     return 'no_result'  # for now this exception is thrown for Hero since speech recognition (meaning his Ears) is not even launched, we don't want it to crash on this

        if self.look_at_standing_person:
            self._robot.head.cancel_goal()

        if answer.sentence in self._options:
            return answer.sentence

        return 'no_result'


class HearOptionsExtra(smach.State):
    """Listen to what the user said, based on a pre-constructed sentence

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
                 time_out=rospy.Duration(10),
                 look_at_standing_person=True):
        smach.State.__init__(self, outcomes=["heard", "no_result"])

        self.robot = robot

        ds.check_resolve_type(spec_designator, str)
        ds.check_resolve_type(speech_result_designator, HMIResult)
        ds.is_writeable(speech_result_designator)

        self.spec_designator = spec_designator
        self.speech_result_designator = speech_result_designator
        self.time_out = time_out
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata=None):
        spec = self.spec_designator.resolve()

        if not spec:
            rospy.logerr("Could not resolve spec")
            return "no_result"

        if self.look_at_standing_person:
            self.robot.head.look_at_standing_person()

        try:
            answer = self.robot.hmi.query('Which option?', spec, 'T',  # TODO: T needs to also be configable
                                          timeout=self.time_out.to_sec())

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

##########################################################################################################################################


class AskContinue(smach.StateMachine):
    def __init__(self, robot, timeout=rospy.Duration(10)):
        smach.StateMachine.__init__(self, outcomes=['continue', 'no_response'])
        self.robot = robot
        self.timeout = timeout

        with self:
            smach.StateMachine.add('SAY',
                                   Say(self.robot,
                                       random.choice(["I will continue my task if you say continue.",
                                                      "Please say continue so that I can continue my task.",
                                                      "I will wait until you say continue."])),
                                   transitions={'spoken': 'HEAR'})

            smach.StateMachine.add('HEAR',
                                   HearOptions(self.robot, ['continue'], self.timeout),
                                   transitions={'continue': 'continue',
                                                'no_result': 'no_response'})


##########################################################################################################################################


class WaitForPersonInFront(smach.State):
    """
    Waits for a person to be found in fron of the robot. Attempts to wait a number of times with a sleep interval
    """

    def __init__(self, robot, attempts=1, sleep_interval=1):
        smach.State.__init__(self, outcomes=["success", "failed"])
        self.robot = robot
        self.attempts = attempts
        self.sleep_interval = sleep_interval

    def execute(self, userdata=None):
        self.robot.head.look_at_standing_person()

        for i in range(self.attempts):
            self._image_data = self._robot.perception.get_rgb_depth_caminfo()
            success, found_people_ids = self._robot.ed.detect_people(*self._image_data)
            if any(found_people_ids):
                return 'success'
            rospy.sleep(rospy.Duration(self.sleep_interval))
        return 'failed'


##########################################################################################################################################


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


##########################################################################################################################################


class WaitForPersonEntity(smach.State):
    """
        Wait until a person is seen/scanned in front of the robot.
            Use paramaterers to costumize number of retries and sleep between retries
    """

    def __init__(self, robot, attempts=1, sleep_interval=1):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.attempts = attempts
        self.sleep_interval = sleep_interval

    def execute(self, userdata=None):
        counter = 0
        detected_humans = None

        while counter < self.attempts:
            print "WaitForPerson: waiting {0}/{1}".format(counter, self.attempts)

            detected_humans = detect_human_in_front(self.robot)
            if detected_humans:
                print "[WaitForPerson] " + "Found a human!"
                return 'succeeded'

            counter += 1
            rospy.sleep(self.sleep_interval)

        return 'failed'


class WaitForPersonDetection(smach.State):
    """
        Wait until a person is seen/scanned in front of the robot.
            Use paramaterers to costumize number of retries and sleep between retries
    """

    def __init__(self, robot, attempts=1, sleep_interval=1):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.attempts = attempts
        self.sleep_interval = sleep_interval

    def execute(self, userdata=None):
        counter = 0
        desgnResult = None

        while counter < self.attempts:
            print "WaitForPerson: waiting {0}/{1}".format(counter, self.attempts)

            rospy.logerr(
                "ed.detect _persons() method disappeared! This was only calling the face recognition module and we are using a new one now!")
            rospy.logerr("I will return an empty detection list!")
            detections = []
            if detections:
                print "[WaitForPersonDetection] " + "Found a human!"
                return 'succeeded'

            counter += 1
            rospy.sleep(self.sleep_interval)

        return 'failed'


##########################################################################################################################################


def detect_human_in_front(robot):
    """
        Scan for humans in the robots field of view. Return person detections if any
    """

    rospy.logerr(
        "ed.detect _persons() method disappeared! This was only calling the face recognition module and we are using a new one now!")
    rospy.logerr("I will return an empty detection list!")
    detections = []

    if not result:
        return False

    for detection in result:
        pose_base_link = detection.pose.projectToFrame(robot.robot_name + '/base_link', robot.tf_listener)

        x = pose_base_link.pose.frame.p.x()
        y = pose_base_link.pose.frame.p.y()

        print "Detection (x,y) in base link: (%f,%f)" % (x, y)

        if 0.0 < x < 1.5 and -1.0 < y < 1.0:
            return True


##########################################################################################################################################


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
            print ("[LearnPersonProcedure] " + "No person found.")
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                print ("[LearnPersonProcedure] " + "Learn procedure timed out!")
                return count

        print ("[LearnPersonProcedure] " + "Completed {0}/{1}".format(count, n_samples))

    print ("[LearnPersonProcedure] " + "Learn procedure completed!")

    # print robot.ed.classify_person(human_id)
    return count


##########################################################################################################################################

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
