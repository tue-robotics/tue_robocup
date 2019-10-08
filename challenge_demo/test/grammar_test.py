from __future__ import print_function
import collections
import unittest

from grammar_parser.cfgparser import CFGParser
from robocup_knowledge import load_knowledge
from robot_skills.mockbot import Mockbot
from action_server.actions.action import ConfigurationResult
from action_server.task_manager import TaskManager

TMTestResult = collections.namedtuple("TMTestResult", ["recipe", "config_result"])


class GrammarTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # import rospy; rospy.init_node("testnode")
        cls.knowledge = load_knowledge('challenge_demo')
        cls.parser = CFGParser.fromstring(cls.knowledge.grammar)
        cls.task_manager = TaskManager(robot=Mockbot())

        # ToDo: Should we make everything recursive???

        # Rule corresponding to the grammar target
        root_rule = cls.parser.rules[cls.knowledge.grammar_target]

        # In principle, a list of actions can be provided to the actions server
        # Therefore, we need to take to steps to list all actions
        # I.e., the following actions_list_target refers to the grammar target
        # containing a list of actions
        actions_list_target = ""
        for option in root_rule.options:
            if "actions" not in option.lsemantic:
                continue
            actions_list_target = option.conjuncts[0].name  # This is too hardcoded
            break

        assert actions_list_target, "Cannot derive actions from grammar"
        actions_list_rule = cls.parser.rules[actions_list_target]
        assert len(actions_list_rule.options) == 1, \
            "Don't know how to continue if the actions list rule has multiple options"
        assert len(actions_list_rule.options[0].conjuncts) == 1, \
            "Don't know how to continue if the actions options list rule has multiple conjunctions"

        # Now, we need to get all separate options
        actions_target = actions_list_rule.options[0].conjuncts[0].name

        cls.actions_rule = cls.parser.rules[actions_target]

    def test_grammar(self):

        test_results = {}
        for option in self.actions_rule.options:

            # Temp for test development
            # if "find" not in option.lsemantic:
            # if "say" not in option.lsemantic:
            # if "hand-over" not in option.lsemantic:
            # if "place" not in option.lsemantic:
            #     continue
            # End of temp thingy

            # option.lsemantic = "'action': 'find', 'object': {'type': X}, 'location': {'id': Y}"
            result_str = ""

            # For each conjunct, we want to have a viable option
            for conjunct in option.conjuncts:
                result_str += self._resolve_conjunct(conjunct.name)
            result_str = result_str.rstrip(" ")
            # print("Result string: <{}>".format(result_str))

            # Now, we can parse the result string to get the action description
            actions_definition = self.parser.parse_raw(
                target=self.knowledge.grammar_target,
                words=result_str,
            )
            self.assertIn(
                "actions",
                actions_definition,
                "'actions' not in actions definition, don't know what to do"
            )

            # print("Action: {}".format(actions_definition))

            # Test the action description with the task manager
            try:
                test_result = self.task_manager.set_up_state_machine(
                    recipe=actions_definition["actions"],
                )  # type: ConfigurationResult
            except Exception as e:
                test_result = ConfigurationResult()
                test_result.message = "Configuration crashed: {}".format(e.message)
            test_results[result_str] = TMTestResult(actions_definition["actions"], test_result)
            # self.assertTrue(
            #     config_result.succeeded,
            #     "Configuration of action '{}' failed.\n\nSentence: '{}'\n\nRecipe: {}\n\nError: {}".format(
            #         actions_definition["actions"][0]["action"],
            #         result_str,
            #         actions_definition,
            #         config_result.message,
            #     ),
            # )

        failed_test_results = {}
        for action, test_result in test_results.iteritems():
            if test_result.config_result.succeeded:
                print("Configuration of '{}' succeeded".format(action))
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
        # knowledge = load_knowledge("challenge_demo")
        # parser = CFGParser.fromstring(knowledge.grammar)

        print("Yeehah, my test has succeeded")

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
            option = rule.options[0]

            for conjunct in option.conjuncts:
                if not conjunct.is_variable:
                    result += conjunct.name
                    result += " "
                else:
                    result += cls._resolve_conjunct(conjunct.name)
        except:
            raise
        return result



if __name__ == "__main__":
    unittest.main()
