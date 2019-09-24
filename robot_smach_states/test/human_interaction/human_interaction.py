# System
import mock
import unittest

# TU/e Robotics
from robot_skills.mockbot import Mockbot

# Robot smach states
import robot_smach_states.util.designators as ds
from robot_smach_states.human_interaction.human_interaction import Say


class TestSay(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def test_say_formatted(self):
        # Simple test
        sentence = "This is an easy sentence"
        say = Say(self.robot, sentence=sentence)
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(sentence, self.robot.speech.speak.call_args_list[-1].args)

        # Provide a list with options that need to be formatted based on the resolution of a designator
        options = ["Hey {a}"]
        options_formatted = [o.format(a="hero") for o in options]
        say = Say(self.robot, options, a=ds.VariableDesignator("hero"))
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(options_formatted[0], self.robot.speech.speak.call_args_list[-1].args)

        # Provide a designator that resolves to a sentence that is formatted with the resolution of a second designator
        des = ds.VariableDesignator(options, resolve_type=[str])
        say = Say(self.robot, des, a=ds.VariableDesignator("hero"))
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(options_formatted[0], self.robot.speech.speak.call_args_list[-1].args)

        # Provide a designator with options that do not need to be formatted and add an superfluous designator
        options = ["Hey"]
        des2 = ds.VariableDesignator(options)
        say = Say(self.robot, des2, a=ds.VariableDesignator("hero"))
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(options[0], self.robot.speech.speak.call_args_list[-1].args)

        # Provide a designator with options that do not need to be formatted
        say = Say(self.robot, des2)
        self.assertEqual(say.execute(), "spoken")
        self.assertIn(options[0], self.robot.speech.speak.call_args_list[-1].args)

        # Provide a wrong keyword argument and check if it fails
        say = Say(self.robot, des, b=ds.VariableDesignator("hero"))
        with self.assertRaises(RuntimeError) as cm:
            say.execute()
            self.assertIn("Not all named place holders are provided", cm.exception)
