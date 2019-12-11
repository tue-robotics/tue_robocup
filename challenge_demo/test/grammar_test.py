import os
import unittest
from ed_msgs.msg import EntityInfo
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import from_entity_info
from action_server.test_tools import test_grammar

from robocup_knowledge import load_knowledge


class GrammarTest(unittest.TestCase):
    @staticmethod
    def test_grammar():
        # Export a (default) robot env. This is necessary because the action server
        # loads knowledge on construction of actions.
        # It is desirable to improve this in the future.
        os.environ["ROBOT_ENV"] = "robotics_testlabs"
        knowledge = load_knowledge('challenge_demo')

        # Construct a Mockbot object and add a number of static entities
        robot = Mockbot()
        robot.ed._static_entities = {
            "couch_table": from_entity_info(EntityInfo(id="couch_table")),
            "operator": from_entity_info(EntityInfo(id="operator")),
        }
        robot.ed._dynamic_entities = dict()
        test_grammar(robot, knowledge.grammar, knowledge.grammar_target)
