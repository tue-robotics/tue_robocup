from __future__ import print_function
import collections
import unittest

from grammar_parser.cfgparser import CFGParser, Conjunct, Option, Rule
from robocup_knowledge import load_knowledge
from robot_skills.mockbot import Mockbot
from action_server.actions.action import ConfigurationResult
from action_server.task_manager import TaskManager

TMTestResult = collections.namedtuple("TMTestResult", ["recipe", "config_result"])


class GrammarTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """
        Loads knowledge of this challenge, constructs the parser and task manager and finds the 'rule' in the grammar
        defining the actions
        """
        cls.knowledge = load_knowledge('challenge_demo')
        cls.parser = CFGParser.fromstring(cls.knowledge.grammar)
        cls.task_manager = TaskManager(robot=Mockbot())

        # ToDo: Should we make everything recursive???

        # Rule corresponding to the grammar target
        root_rule = cls.parser.rules[cls.knowledge.grammar_target]  # type: Rule

        # In principle, a list of actions can be provided to the actions server
        # Therefore, we need to take to steps to list all actions
        # I.e., the following actions_list_target refers to the grammar target
        # containing a list of actions
        actions_list_target = ""
        for option in root_rule.options:  # type: Option
            if "actions" not in option.lsemantic:
                continue
            actions_list_target = option.conjuncts[0].name  # type: str# This is too hardcoded
            break

        assert actions_list_target, "Cannot derive actions from grammar"
        actions_list_rule = cls.parser.rules[actions_list_target]  # type: Rule
        assert len(actions_list_rule.options) == 1, \
            "Don't know how to continue if the actions list rule has multiple options"
        assert len(actions_list_rule.options[0].conjuncts) == 1, \
            "Don't know how to continue if the actions options list rule has multiple conjunctions"

        # Now, we need to get all separate options
        actions_target = actions_list_rule.options[0].conjuncts[0].name  # type: str

        cls.actions_rule = cls.parser.rules[actions_target]  # type: Rule

    def test_grammar(self):
        """
        Based on the actions rule, it recursively determines a viable option (e.g., 'go to the couch_table'), parses
        this using the CFGParser and tries to configure the task manager based on the parse result. All results (even
        if the configuration step with the task manager raises an exception) are stored in a result dict. Eventually,
        all positive results are printed and if negative results have occurred, an AssertionError is raised with a
        message containing all failed options.
        """

        test_results = {}
        for option in self.actions_rule.options:  # type: Option

            result_str = ""

            # For each conjunct, we want to have a viable option
            for conjunct in option.conjuncts:  # type: Conjunct
                result_str += self._resolve_conjunct(conjunct.name)
            result_str = result_str.rstrip(" ")  # type: str

            # Now, we can parse the result string to get the action description
            actions_definition = self.parser.parse_raw(
                target=self.knowledge.grammar_target,
                words=result_str,
            )  # type: dict
            self.assertIn(
                "actions",
                actions_definition,
                "'actions' not in actions definition, don't know what to do"
            )

            # Test the action description with the task manager
            try:
                test_result = self.task_manager.set_up_state_machine(
                    recipe=actions_definition["actions"],
                )  # type: ConfigurationResult
            except Exception as e:
                test_result = ConfigurationResult()
                test_result.message = "Configuration crashed: {}".format(e.message)
            test_results[result_str] = TMTestResult(actions_definition["actions"], test_result)

        failed_test_results = {}
        for action, test_result in test_results.iteritems():
            if test_result.config_result.succeeded:
                print("Configuration of '{}' succeeded".format(action))
            elif test_result.config_result.message and test_result.config_result.missing_field:
                print("Configuration of '{}' did not succeed due to missing information ({}): {}".format(
                    action, test_result.config_result.missing_field, test_result.config_result.message
                ))
            else:
                failed_test_results[action] = test_result

        error_str = "\n"
        for action, test_result in failed_test_results.iteritems():
            error_str += "\nConfiguration of action '{}' failed.\n\tRecipe: {}\n\tError: {}".format(
                action,
                test_result.recipe,
                test_result.config_result.message,
            )
        self.assertFalse(failed_test_results, error_str)

    @classmethod
    def _resolve_conjunct(cls, target):
        # type: (str) -> str
        """
        Recursive method that tries to resolve a conjunct, i.e., to find an option

        :param target: target to look for
        :return: option string
        """
        if target not in cls.parser.rules:
            return target + " "

        result = ""
        rule = cls.parser.rules[target]  # type: Rule
        # For now, take the first available option
        # If we want to make this method entirely recursive (from the root of the grammar), we might want to cook up
        # something more generic.
        option = rule.options[0]  # type: Option

        for conjunct in option.conjuncts:  # type: Conjunct
            if not conjunct.is_variable:
                result += conjunct.name
                result += " "
            else:
                result += cls._resolve_conjunct(conjunct.name)
        return result


if __name__ == "__main__":
    unittest.main()
