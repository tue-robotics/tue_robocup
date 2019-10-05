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


        pass

    def test_grammar(self):

        for option in self.actions_rule.options:
            print("Action: {}".format(option.conjuncts[0].name))
        # knowledge = load_knowledge("challenge_demo")
        # parser = CFGParser.fromstring(knowledge.grammar)

        print("Yeehah, my test has succeeded")


if __name__ == "__main__":
    unittest.main()
