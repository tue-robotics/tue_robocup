import unittest
from action_server.test_tools import test_grammar

from robocup_knowledge import load_knowledge


class GrammarTest(unittest.TestCase):
    @staticmethod
    def test_grammar():
        knowledge = load_knowledge('challenge_demo')
        test_grammar(knowledge.grammar, knowledge.grammar_target)
