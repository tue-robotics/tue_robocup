from __future__ import print_function
import unittest

from grammar_parser.cfgparser import CFGParser
from robocup_knowledge import load_knowledge


class GrammarTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.knowledge = load_knowledge('challenge_demo')
        cls.parser = CFGParser.fromstring(cls.knowledge.grammar)

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
        assert (len(actions_list_rule.options) == 1,
                "Don't know how to continue if the actions list rule has multiple options")
        assert (len(actions_list_rule.options[0].conjuncts) == 1,
                "Don't know how to continue if the actions list rule has multiple options")

        # Now, we need to get all separate options
        actions_target = actions_list_rule.options[0].conjuncts[0].name

        cls.actions_rule = cls.parser.rules[actions_target]

    def test_grammar(self):

        for option in self.actions_rule.options:

            # Temp for test development
            if "find" not in option.lsemantic:
                continue
            # End of temp thingy

            # option.lsemantic = "'action': 'find', 'object': {'type': X}, 'location': {'id': Y}"
            result_str = ""

            # For each conjunct, we want to have a viable option
            for conjunct in option.conjuncts:
                result_str += self._resolve_conjunct(conjunct.name)
            result_str = result_str.rstrip(" ")
            print("Result string: <{}>".format(result_str))

            # Now, we can parse the result string to get the action description
            action_definition = self.parser.parse_raw(
                target=self.knowledge.grammar_target,
                words=result_str,
            )

            print("Action: {}".format(action_definition))

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

        return result



if __name__ == "__main__":
    unittest.main()
