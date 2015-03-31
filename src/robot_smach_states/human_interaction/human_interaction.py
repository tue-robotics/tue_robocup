#! /usr/bin/env python
import roslib;
import rospy
import smach
import random
import ed_perception.msg
# from ed_perception.srv import LearnPerson, LearnPersonRequest
import actionlib #
# import actionlib_msgs
from robot_smach_states.state import State

from robot_smach_states.util.designators import Designator, DesignatorResolvementError, EdEntityDesignator, check_type
from robot_smach_states.utility import WaitForDesignator
import robot_skills.util.msg_constructors as gm
from smach_ros import SimpleActionState
from ed_perception.msg import FaceLearningGoal, FaceLearningResult #


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
    >>> robot.speech.speak.assert_any_call('a', 'us', 'kyle', 'default', 'excited', True)
    >>> robot.speech.speak.assert_any_call('b', 'us', 'kyle', 'default', 'excited', True)
    >>> robot.speech.speak.assert_any_call('c', 'us', 'kyle', 'default', 'excited', True)"""
    def __init__(self, robot, sentence=None, language="us", personality="kyle", voice="default", mood="excited", block=True):
        check_type(sentence, str, list)
        check_type(sentence, str, list)
        check_type(language, str)
        check_type(personality, str)
        check_type(voice, str)
        check_type(mood, str)
        check_type(block, bool)

        State.__init__(self, locals(), outcomes=["spoken"])

    def run(self, robot, sentence, language, personality, voice, mood, block):
        robot.head.look_at_standing_person()

        if not isinstance(sentence, str) and isinstance(sentence, list):
            sentence = random.choice(sentence)

        robot.speech.speak(sentence, language, personality, voice, mood, block)

        robot.head.cancel_goal()

        return "spoken"

##########################################################################################################################################

class Hear(State):
    def __init__(self, robot, spec, time_out = rospy.Duration(10)):
        State.__init__(self, locals(), outcomes=["heard", "not_heard"])

    def run(self, robot, spec, time_out):
        robot.head.look_at_standing_person()

        answer = robot.ears.recognize(spec, {}, time_out)

        robot.head.cancel_goal()

        if answer:
            if answer.result:
                return "heard"
        else:
            robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "not_heard"

class HearOptions(State):
    def __init__(self, robot, options, time_out = rospy.Duration(10)):
        State.__init__(self, locals(), outcomes=options.append("no_result"))

    def run(self, robot, options, time_out):
        robot.head.look_at_standing_person()

        answer = robot.ears.recognize("<option>", {"option":options}, time_out)

        robot.head.cancel_goal()

        if answer:
            if answer.result:
                return answer.choices["option"]
        else:
            robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "no_result"

class HearOptionsExtra(smach.State):
    """
    >>> from robot_skills.mockbot import Mockbot
    >>> mockbot = Mockbot()
    >>> from robot_smach_states.util.designators import Designator, VariableDesignator
    >>> spec = Designator("I will go to the <table> in the <room>")
    >>> choices = Designator({  "room"  : ["livingroom", "bedroom", "kitchen" ], "table" : ["dinner table", "couch table", "desk"]})
    >>> answer = VariableDesignator()
    >>> state = HearOptionsExtra(mockbot, spec, choices, answer)
    >>> outcome = state.execute()
    """
    def __init__(self, robot, spec_designator,
                        choices_designator,
                        speech_result_designator,
                        time_out=rospy.Duration(10)):
        smach.State.__init__(self, outcomes=["heard", "no_result"])

        self.robot = robot
        self.spec_designator = spec_designator
        self.choices_designator = choices_designator
        self.speech_result_designator = speech_result_designator
        self.time_out = time_out

    def execute(self, userdata=None):
        spec = self.spec_designator.resolve()
        choices = self.choices_designator.resolve()

        self.robot.head.look_at_standing_person()

        answer = self.robot.ears.recognize(spec, choices, self.time_out)

        self.robot.head.cancel_goal()

        if answer:
            if answer.result:
                self.speech_result_designator.current = answer
                return "heard"
        else:
            self.robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "no_result"


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
                                    transitions={'heard':'continue','not_heard':'no_response'})

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


class LearnPerson(smach.StateMachine):
    '''
    State that provides an interface to learn a person's face.

    person_name - name of the person to be learned
    name_over_userdata - Enables the passing of the name through userdata. If set to true, person_name is ignored

    tutorial for SimpleActionState here http://wiki.ros.org/smach/Tutorials/SimpleActionState
    '''
    def __init__(self, robot, person_name = "", name_over_userdata = False):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeded_learning', 'failed_learning'],
                                    input_keys=['personName_in'])
        self.robot = robot
        self.service_name = "/" + robot.robot_name + "/ed/face_recognition/learn_face"

        with self:

            # Callback when a result is received
            def learn_result_cb(userdata, status, result):
                print "Received result from the learning service"

                # test the result and parse the message
                if status == actionlib.GoalStatus.SUCCEEDED:
                    if result.result_info == "Learning complete":
                        print "Face learning complete! result: " + result.result_info
                        # self.robot.speech.speak("Learning complete.", block=False)

                        return 'succeeded'
                    else:
                        return 'aborted'
                else:
                    print "Face learning aborted! result: " + result.result_info
                    return 'aborted'

            # --------------------------------------------------------------------------------

            # Callback when a result is sent
            def learn_goal_cb(userdata, goal):
                # import ipdb; ipdb.set_trace()

                # self.robot.speech.speak("Please look at me while I learn your face.", block=True)

                learn_goal = FaceLearningGoal()
                learn_goal.person_name = userdata.person_name_goal

                print "Goal sent to the learning service, with name '" + learn_goal.person_name + "'"

                return learn_goal

            # --------------------------------------------------------------------------------

            # Create Simple Action ClientS
            smach.StateMachine.add( 'LEARN_PERSON',
                                    SimpleActionState(  self.service_name,
                                                        ed_perception.msg.FaceLearningAction,
                                                        result_cb = learn_result_cb,
                                                        goal_cb = learn_goal_cb,            # create a goal inside the callback
                                                        input_keys=['person_name_goal'],
                                                        output_keys=['result_info_out']),
                                                        # goal_slots = ['person_name_goal'],# or create it here directly
                                    transitions={   'succeeded':'succeded_learning',
                                                    'aborted': 'failed_learning',
                                                    'preempted': 'failed_learning'},
                                    remapping={     'person_name_goal':'personName_in'})



##########################################################################################################################################

if __name__ == "__main__":
    import doctest
    doctest.testmod()
