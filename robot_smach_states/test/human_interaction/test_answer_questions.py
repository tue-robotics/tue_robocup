# System
import unittest

# TU/e Robotics
from robot_skills.mockbot import Mockbot

# Robot smach states
from robot_smach_states.human_interaction.answer_questions import HearAndAnswerQuestions


class TestHearAnswerQuestions(unittest.TestCase):

    def setUp(self):
        self.robot = Mockbot()

    def test_teams_name(self):
        grammar = """
        T[{actions : <A1>}] -> C[A1]

        C[{A}] -> Q[A]
        """

        grammar += """
        Q["action" : "answer", "solution": "Tech United"] -> what is the name of your team
        """
        state = HearAndAnswerQuestions(
            robot=self.robot,
            grammar=grammar,
            knowledge=None,
        )
        state.execute({})
        self.assertIn("tech united", self.robot.speech.speak.mock_calls[0].args[0].lower())


if __name__ == '__main__':
    unittest.main()
