#! /usr/bin/env python
import roslib;
import rospy
import smach
import random
import ed_perception.msg
import actionlib
from robot_smach_states.state import State

from robot_smach_states.util.designators import Designator, EdEntityDesignator, VariableDesignator, check_type, check_resolve_type
from robot_smach_states.utility import WaitForDesignator
import robot_skills.util.msg_constructors as gm
from smach_ros import SimpleActionState
from ed_perception.msg import FaceLearningGoal, FaceLearningResult #
from dragonfly_speech_recognition.srv import GetSpeechResponse


# Say: Immediate say
# Hear: Immediate hear
# Ask: Interaction, say + hear

##########################################################################################################################################

class Say(State):
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
    def __init__(self, robot, sentence=None, language=None, personality=None, voice=None, mood=None, block=True):
        check_type(sentence, str, list)
        #check_type(language, str)
        #check_type(personality, str)
        #check_type(voice, str)
        #check_type(mood, str)
        check_type(block, bool)

        State.__init__(self, locals(), outcomes=["spoken"])

    def run(self, robot, sentence, language, personality, voice, mood, block):
        #robot.head.look_at_standing_person()

        if not sentence:
            rospy.logerr("sentence = None, not saying anything...")
            return "spoken"

        if not isinstance(sentence, str) and isinstance(sentence, list):
            sentence = random.choice(sentence)

        robot.speech.speak(sentence, language, personality, voice, mood, block)

        #robot.head.cancel_goal()

        return "spoken"

##########################################################################################################################################

class Hear(State):
    def __init__(self, robot, spec, time_out = rospy.Duration(10), look_at_standing_person=True):
        State.__init__(self, locals(), outcomes=["heard", "not_heard"])

    def run(self, robot, spec, time_out, look_at_standing_person):
        if look_at_standing_person:
            robot.head.look_at_standing_person()

        answer = robot.ears.recognize(spec, {}, time_out)

        if look_at_standing_person:
            robot.head.cancel_goal()

        if answer:
            if answer.result:
                return "heard"
        else:
            robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "not_heard"

class HearOptions(smach.State):
    def __init__(self, robot, options, timeout = rospy.Duration(10), look_at_standing_person=True):
        outcomes = options
        outcomes.append("no_result")
        smach.State.__init__(self, outcomes=outcomes)
        self._options = options
        self._robot = robot
        self._timeout = timeout
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata):
        if self.look_at_standing_person:
            self._robot.head.look_at_standing_person()

        answer = self._robot.ears.recognize("<option>", {"option":self._options}, self._timeout)

        if self.look_at_standing_person:
            self._robot.head.cancel_goal()

        if answer:
            if answer.result:
                return answer.choices["option"]
        else:
            self._robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "no_result"

class HearOptionsExtra(smach.State):
    """Listen to what the user said, based on a pre-constructed sentence

    Keyword arguments:
    spec_designator -- sentence that is supposed to be heard
    choices_designator -- list of choices for words in the sentence
    speech_result_designator -- variable where the result is stored
    time_out -- timeout in case nothing is heard

    Example of usage:
        from dragonfly_speech_recognition.srv import GetSpeechResponse
        spec = Designator("((<prefix> <name>)|<name>)")
        choices = Designator({"name"  : names_list,
                              "prefix": ["My name is", "I'm called"]})
        answer = VariableDesignator(resolve_type = GetSpeechResponse)
        state = HearOptionsExtra(self.robot, spec, choices, answer)
        outcome = state.execute()

        if outcome == "heard":
            name = answer.resolve().choices["name"]

    >>> from robot_skills.mockbot import Mockbot
    >>> mockbot = Mockbot()
    >>> from robot_smach_states.util.designators import Designator, VariableDesignator
    >>> spec = Designator("I will go to the <table> in the <room>")
    >>> choices = Designator({  "room"  : ["livingroom", "bedroom", "kitchen" ], "table" : ["dinner table", "couch table", "desk"]})
    >>> answer = VariableDesignator(resolve_type=GetSpeechResponse)
    >>> state = HearOptionsExtra(mockbot, spec, choices, answer)
    >>> outcome = state.execute()
    """
    def __init__(self, robot, spec_designator,
                        choices_designator,
                        speech_result_designator,
                        time_out=rospy.Duration(10),
                        look_at_standing_person=True):
        smach.State.__init__(self, outcomes=["heard", "no_result"])

        self.robot = robot

        check_resolve_type(spec_designator, str)
        check_resolve_type(choices_designator, dict)
        check_resolve_type(speech_result_designator, GetSpeechResponse)

        self.spec_designator = spec_designator
        self.choices_designator = choices_designator
        self.speech_result_designator = speech_result_designator
        self.time_out = time_out
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata=None):
        spec = self.spec_designator.resolve()
        choices = self.choices_designator.resolve()

        if not spec:
            rospy.logerr("Could not resolve spec")
            return "no_result"
        if not choices:
            rospy.logerr("Could not resolve choices")
            return "no_result"


        if self.look_at_standing_person:
            self.robot.head.look_at_standing_person()

        answer = self.robot.ears.recognize(spec, choices, self.time_out)

        if self.look_at_standing_person:
            self.robot.head.cancel_goal()

        if answer:
            if answer.result:
                self.speech_result_designator.current = answer
                return "heard"
        else:
            self.robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "no_result"


##########################################################################################################################################


class HearYesNo(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['heard_yes', 'heard_no', 'heard_failed'])

        self.robot = robot

    def execute(self, userdata):
        # define answer format 
        spec = Designator("(<positive_answer>|<negative_answer>)")

        # define choices
        choices = Designator({  "positive_answer": ["Yes", "Correct", "Right", "Yup"],
                                "negative_answer": ["No", "Incorrect", "Wrong", "Nope"]})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, spec, choices, answer)
        
        # execute listen
        outcome = state.execute()

        if not outcome == "heard":
            # if there was no answer
            print "HearYesNo: did not hear anything!"
            return 'heard_failed'
        else:
            response_negative = ""
            response_positive = ""

            # test if the answer was positive, if its empty it will return excepton and continue to negative answer
            try:
                response_positive = answer.resolve().choices["positive_answer"]
                
                print "HearYesNo: answer is positive, heard: '" + response_positive + "'"
                return 'heard_yes'
            except KeyError, ke:
                print "KeyError resolving the answer heard: " + str(ke)
                pass

            try:
                response_negative = answer.resolve().choices["negative_answer"]
                
                print "HearYesNo: answer is negative, heard: '" + response_negative + "'"
                return 'heard_no'
            except KeyError, ke:
                print "KeyError resolving the answer heard: " + str(ke)
                pass

        print "HearYesNo: could not resolve answer!"

        return 'heard_failed'


##########################################################################################################################################

class AskContinue(smach.StateMachine):
    def __init__(self, robot, timeout=rospy.Duration(10)):
        smach.StateMachine.__init__(self, outcomes=['continue','no_response'])
        self.robot = robot
        self.timeout = timeout

        with self:
            smach.StateMachine.add('SAY',
                                    Say(self.robot,
                                        random.choice(["I will continue my task if you say continue.","Please say continue so that I can continue my task.","I will wait until you say continue."])),
                                    transitions={'spoken':'HEAR'})

            smach.StateMachine.add('HEAR',
                                    Hear(self.robot, 'continue', self.timeout),
                                    transitions={   'heard':'continue',
                                                    'not_heard':'no_response'})

##########################################################################################################################################

class WaitForPersonInFront(WaitForDesignator):
    """
    Waits for a person to be found in fron of the robot. Attempts to wait a number of times with a sleep interval
    """

    def __init__(self, robot, attempts = 1, sleep_interval = 1):
        # TODO: add center_point in front of the robot and radius of the search on EdEntityDesignator
        # human_entity = EdEntityDesignator(robot, center_point=gm.PointStamped(x=1.0, frame_id="base_link"), radius=1, id="human")
        human_entity = EdEntityDesignator(robot, type="human")
        WaitForDesignator.__init__(self, robot, human_entity, attempts, sleep_interval)


##########################################################################################################################################

class NameToUserData(WaitForDesignator):
    """
    Pass the received name into userdata. By default use 'person_name', if its empty use a designator
    """
    def __init__(self, robot, person_name = "", name_designator = None):
        smach.State.__init__(   self,
                                output_keys=['personName_out'],
                                outcomes=['done'])

        self.robot = robot
        self.person_name = person_name
        self.name_designator = name_designator

    def execute(self, userdata):
        print "NameToUserData"
        
        name = ""
        # if the name was given in person_name parameter use it, otherwise use designator
        if self.person_name:
            name = self.person_name
        elif self.name_designator:    
            name = self.name_designator.resolve()
        else:
            name = "Default"
            print "ERROR: Could not get the goal name for the person! Using '" + name + "'"

        userdata.personName_out = name
        print "NameToUserData: param -> {0}".format(name)

        # import ipdb; ipdb.set_trace()
        return 'done'


##########################################################################################################################################


class LearnPerson(smach.StateMachine):
    """
    State that provides an interface to learn a person's face through actionlib call

    Keyword arguments:
    person_name -- name of the person to be learned

    tutorial for SimpleActionState here http://wiki.ros.org/smach/Tutorials/SimpleActionState
    """
    def __init__(self, robot, person_name = "", name_designator = None):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeded_learning', 'failed_learning'])
        self.robot = robot
        self.person_name = person_name
        self.name_designator = name_designator
        self.service_name = "/" + robot.robot_name + "/ed/face_recognition/learn_face"

        with self:

            # Callback when a result is received
            def get_result_cb(userdata, status, result):
                print "Received result from the learning service"

                # test the result and parse the message
                if status == actionlib.GoalStatus.SUCCEEDED:
                    if result.result_info == "Learning complete":
                        print "Face learning complete! result: " + result.result_info
                        return 'succeeded'
                    else:
                        return 'aborted'
                else:
                    print "Face learning aborted! result: " + result.result_info
                    return 'aborted'

            # --------------------------------------------------------------------------------

            # Callback when a result is sent
            def send_goal_cb(userdata, goal):
                # import ipdb; ipdb.set_trace()

                learn_goal = FaceLearningGoal()
                learn_goal.person_name = userdata.person_name_goal

                print "Goal sent to the learning service (" + self.service_name + "), with name '" + learn_goal.person_name + "'"

                return learn_goal

            # --------------------------------------------------------------------------------

            smach.StateMachine.add( 'NAME_TO_USERDATA',
                                    NameToUserData(robot, person_name = self.person_name, name_designator = self.name_designator),
                                    remapping={     'personName_out':'personName_userdata'},
                                    transitions={   'done': 'LEARN_PERSON'})

            # Create Simple Action ClientS
            smach.StateMachine.add( 'LEARN_PERSON',
                                    SimpleActionState(  self.service_name,
                                                        ed_perception.msg.FaceLearningAction,
                                                        result_cb = get_result_cb,
                                                        goal_cb = send_goal_cb,            # create a goal inside the callback
                                                        input_keys=['person_name_goal'],
                                                        output_keys=['result_info_out']),
                                                        # goal_slots = ['person_name_goal'],# or create it here directly
                                    transitions={   'succeeded':'succeded_learning',
                                                    'aborted': 'failed_learning',
                                                    'preempted': 'failed_learning'},
                                    remapping={     'person_name_goal':'personName_userdata'})



##########################################################################################################################################

if __name__ == "__main__":
    import doctest
    doctest.testmod()
