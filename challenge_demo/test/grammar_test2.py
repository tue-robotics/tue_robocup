from __future__ import print_function

import re
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

    @unittest.skip("This probably ain't going to work")
    def test_grammar(self):
        # target_rule = self.parser.rules[self.knowledge.grammar_target]
        options = self._expand_conjunct(self.knowledge.grammar_target, 3)
        result_str = self._resolve_conjunct(self.knowledge.grammar_target)
        print("Yeehah, I'm done")

        #     unwrapped = self.parser.get_unwrapped(self.knowledge.grammar_target)
        #     spec = "(%s)" % unwrapped
        #     options = self._expand_options(spec, target_depth=5)
        #     resolved_options = []
        #     for idx, option in enumerate(options):
        #         resolved_options.append(self._resolve_spec(option))
        #     spec = self._resolve_spec(spec)

    @classmethod
    def _expand_conjunct(cls, target, target_depth, current_depth=0):
        """
        Recursive method that tries to expand a conjunct, i.e., to list all options

        :param target: target to expand
        :param target_depth: depth to expand to recursively
        :return: list with options
        """
        # ToDo: make target depth class attribute?
        result = []
        if target not in cls.parser.rules:
            return result

        rule = cls.parser.rules[target]
        current_depth += 1
        for option in rule.options:
            for conj in option.conjuncts:
                if target_depth == current_depth:
                    result.append(conj)
                else:
                    result += cls._expand_conjunct(conj.name, target_depth, current_depth)

        return result

    @classmethod
    def _resolve_conjunct(cls, target):
        # type: (str) -> str
        """
        Recursive method that tries to resolve a conjunct, i.e., to find an option

        :param target: target to look for
        :return: option string
        """
        try:  # ToDo: remove try except
            if target not in cls.parser.rules:
                return target + " "

            result = ""
            rule = cls.parser.rules[target]
            # For now, take the first available option
            if isinstance(rule.options, list):
                option = rule.options[0]
            else:
                option = rule.options

            for conjunct in option.conjuncts:
                if not conjunct.is_variable:
                    result += conjunct.name
                    result += " "
                else:
                    result += cls._resolve_conjunct(conjunct.name)
        except:
            raise
        return result

    @staticmethod
    def _is_fixed_option(option):
        """
         checks if this option is 'fixed', i.e.,
        :return:
        """
        if isinstance(option, str):
            return True
        return all([not conj.is_variable for conj in option.conjuncts])


    #
    # def _resolve_spec(self, spec):
    #
    #     while re.search('\([^)]+\)', spec):
    #         spec = self._resolve_options(spec)
    #         # options = re.findall('\([^()]+\)', spec)
    #         # for option in options:
    #         #     # spec = spec.replace(option, random.choice(option[1:-1].split("|")), 1)
    #         #     spec = spec.replace(option, option[1:-1].split("|")[0], 1)
    #
    #     return spec
    #
    # def _resolve_options(self, spec):
    #     options = re.findall('\([^()]+\)', spec)
    #     for option in options:
    #         spec = spec.replace(option, option[1:-1].split("|")[0], 1)
    #
    #     return spec
    #
    # def _expand_options(self, input_spec, target_depth=1):
    #     input_list = [input_spec]
    #     for _ in range(target_depth):
    #         output_list = []
    #         for spec in input_list:
    #             options = re.findall('\([^()]+\)', spec)
    #             for option in options:
    #                 idx = 0
    #                 while True:
    #                     try:
    #                         output_list.append(spec.replace(option, option[1:-1].split("|")[idx], 1))
    #                     except IndexError:
    #                         break
    #                     idx += 1
    #         input_list = output_list
    #     return output_list
