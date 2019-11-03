# System
import unittest

# TU/e Robotics
from hmi import TimeoutException, HMIResult
from robot_skills.mockbot import Mockbot

# Robot smach states
import robot_smach_states.util.designators as ds
from robot_smach_states.human_interaction.human_interaction import Say, HearOptions, WaitForPersonInFront, AskYesNo


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

    def test_ask_yes_no(self):
        state = AskYesNo(self.robot)

        for option in ["yes", "no"]:
            self.robot.hmi.query = lambda description, grammar, target, timeout: HMIResult(sentence=option,
                                                                                           semantics=option)
            self.assertEqual(state.execute(), option)


class TestWaitForPerson(unittest.TestCase):
    def test_wait_for_person(self):
        robot = Mockbot()

        def _detect_people_fail(*args, **kwargs):
            return False, []

        def _detect_people_success(*args, **kwargs):
            return True, ["abc", "def"]

        nr_attempts = 2
        state = WaitForPersonInFront(robot, attempts=nr_attempts, sleep_interval=0.01)

        # Check failing
        robot.ed.detect_people = _detect_people_fail
        result = state.execute()
        self.assertEqual(result, "failed", "WaitForPersonInFront succeeded while it should have failed")
        nr_calls = robot.perception.get_rgb_depth_caminfo.call_count
        self.assertEqual(nr_calls, nr_attempts,
                         "Perception only called {} instead of {}".format(nr_calls, nr_attempts),
                         )

        # Check succeeding
        robot.ed.detect_people = _detect_people_success
        result = state.execute()
        self.assertEqual(result, "success", "WaitForPersonInFront failed while it should have succeeded")


if __name__ == '__main__':
    unittest.main()
