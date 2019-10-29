# System
import unittest

# TU/e Robotics
from hmi import TimeoutException
from robot_skills.mockbot import Mockbot

# Robot smach states
import robot_smach_states.util.designators as ds
from robot_smach_states.human_interaction.human_interaction import Say, HearOptions


class TestSay(unittest.TestCase):

    def setUp(self):
        self.robot = Mockbot()

    def test_say(self):
        # Simple test
        sentence = "This is an easy sentence"
        say = Say(self.robot, sentence=sentence)
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(sentence, self.robot.speech.speak.call_args_list[-1].args)

        # Provide a list with options that need to be formatted based on the resolution of a designator
        option = "Hey {a}"
        formatted_option = option.format(a="hero")
        say = Say(self.robot, [option], a=ds.VariableDesignator("hero"))
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(formatted_option, self.robot.speech.speak.call_args_list[-1].args)

        # Provide a designator that resolves to a sentence that is formatted with the resolution of a second designator
        des = ds.VariableDesignator([option], resolve_type=[str])
        say = Say(self.robot, des, a=ds.VariableDesignator("hero"))
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(formatted_option, self.robot.speech.speak.call_args_list[-1].args)

        # Provide a designator with options that do not need to be formatted and add an superfluous designator
        option = "Hey"
        des2 = ds.VariableDesignator([option])
        say = Say(self.robot, des2, a=ds.VariableDesignator("hero"))
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(option, self.robot.speech.speak.call_args_list[-1].args)

        # Provide a designator with options that do not need to be formatted
        say = Say(self.robot, des2)
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(option, self.robot.speech.speak.call_args_list[-1].args)

        # Provide a wrong keyword argument and check if it fails
        say = Say(self.robot, des, b=ds.VariableDesignator("hero"))
        with self.assertRaises(RuntimeError) as cm:
            say.execute()
            self.assertIn("Not all named place holders are provided", cm.exception)


class TestHearOptions(unittest.TestCase):

    def setUp(self):
        self.robot = Mockbot()

    def test_hear_options_success(self):
        options = ["apple"]
        hear = HearOptions(self.robot, options)
        self.assertEqual(hear.execute(), "apple")

    def test_hear_options_fail(self):
        class HMIMock(object):
            def query(self, description, target, grammar, timeout):
                raise TimeoutException

        options = ["apple"]
        self.robot.hmi = HMIMock()
        hear = HearOptions(self.robot, options)
        self.assertEqual(hear.execute(), "no_result")

