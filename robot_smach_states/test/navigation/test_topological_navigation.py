# System
import unittest

# TU/e Robotics
from robot_skills import get_robot
from topological_action_planner_msgs.msg import Edge, Node

# Robot Smach States
from robot_smach_states.manipulation.open_door import OpenDoor
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic
from robot_smach_states.navigation.navigate_to_waypoint import NavigateToWaypoint
from robot_smach_states.navigation.topological_navigation import TopologicalPlannerException, convert_msgs_to_actions

SOURCE_NODE_ID = "SOURCE_NODE"
SOURCE_NODE_AREA = "IN_FRONT_OF"
DESTINATION_NODE_ID = "DESTINATION_NODE"
DESTINATION_NODE_AREA = "IN_FRONT_OF"


class TestTopologicalNavigation(unittest.TestCase):
    """
    Tests the TopologicalNavigation smach state
    """

    @classmethod
    def setUpClass(cls):
        cls.robot = get_robot("mockbot")

    def test_drive(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_DRIVE
        msg.origin = Node(SOURCE_NODE_ID, SOURCE_NODE_AREA)
        msg.destination = Node(DESTINATION_NODE_ID, DESTINATION_NODE_AREA)
        actions = convert_msgs_to_actions(self.robot, [msg])
        self.assertEqual(len(actions), 1, "Result should contain exactly one action")
        self.assertTrue(isinstance(actions[0], NavigateToSymbolic), "Action should be a 'NavigateToSymbolic' action")

    def test_drive_waypoint(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_DRIVE
        msg.origin = Node(SOURCE_NODE_ID, SOURCE_NODE_AREA)
        msg.destination.entity = DESTINATION_NODE_ID
        actions = convert_msgs_to_actions(self.robot, [msg])
        self.assertEqual(len(actions), 1, "Result should contain exactly one action")
        self.assertTrue(isinstance(actions[0], NavigateToWaypoint), "Action should be a 'NavigateToWaypoint' action")

    def test_open_door(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_OPEN_DOOR
        entity_id = "door"
        msg.origin = Node(entity_id, SOURCE_NODE_AREA)
        msg.destination = Node(entity_id, DESTINATION_NODE_AREA)
        actions = convert_msgs_to_actions(self.robot, [msg])
        self.assertEqual(len(actions), 1, "Result should contain exactly one action")
        self.assertTrue(isinstance(actions[0], OpenDoor), "Action should be a 'OpenDoor' action")

    @unittest.skip
    def test_open_door_wrong_entity(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_USE_ELEVATOR
        entity_id = "door"
        msg.origin = Node(SOURCE_NODE_ID, SOURCE_NODE_AREA)
        msg.destination = Node(entity_id, DESTINATION_NODE_AREA)
        self.assertRaises(TopologicalPlannerException, convert_msgs_to_actions, self.robot, [msg])

    @unittest.skip
    def test_open_door_wrong_area(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_OPEN_DOOR
        entity_id = "door"
        msg.origin = Node(entity_id, "")
        msg.destination = Node(entity_id, DESTINATION_NODE_AREA)
        self.assertRaises(TopologicalPlannerException, convert_msgs_to_actions, self.robot, [msg])

    def test_invalid_action(self):
        msg = Edge()
        msg.action_type = "foo"
        self.assertRaises(TopologicalPlannerException, convert_msgs_to_actions, self.robot, [msg])
