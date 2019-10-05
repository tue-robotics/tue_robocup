from __future__ import print_function
import unittest

from grammar_parser.cfgparser import CFGParser
from robocup_knowledge import load_knowledge
from robot_skills.mockbot import Mockbot
from action_server.actions.action import ConfigurationResult
from action_server.task_manager import TaskManager


class GrammarTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):

        # Load the knowledge, grammar parser and task manager
        cls.knowledge = load_knowledge('challenge_demo')
        cls.parser = CFGParser.fromstring(cls.knowledge.grammar)
        cls.task_manager = TaskManager(robot=Mockbot())

        # Try to prune the grammar to get rid of similar options.
        # E.g., if we have tested "bring me the coke", we don't want to test "bring me the beer"
        for target, rule in cls.parser.rules.iteritems():
            if all([cls._is_fixed_option(option) for option in rule.options]):
                rule.options = rule.options[0]
        pass

    def test_grammar(self):
        pass

    @staticmethod
    def _is_fixed_option(option):
        """
         checks if this option is 'fixed', i.e.,
        :return:
        """
        if isinstance(option, str):
            return True
        return all([not conj.is_variable for conj in option.conjuncts])
