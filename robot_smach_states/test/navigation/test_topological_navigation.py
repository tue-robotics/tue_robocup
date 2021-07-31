# System
import mock
import unittest

# TU/e Robotics
from topological_action_planner_msgs.msg import Edge, Node

# Robot Smach States
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic
from robot_smach_states.navigation.navigate_to_waypoint import NavigateToWaypoint
from robot_smach_states.navigation.navigation import NavigateTo
from robot_smach_states.navigation.topological_navigation import TopologicalPlannerException, convert_msgs_to_actions

SOURCE_NODE_ID = "SOURCE_NODE"
SOURCE_NODE_AREA = "IN_FRONT_OF"
DESTINATION_NODE_ID = "DESTINATION_NODE"
DESTINATION_NODE_AREA = "IN_FRONT_OF"
ROBOT = mock.Mock()


class TestTopologicalNavigation(unittest.TestCase):
    """
    Tests the TopologicalNavigation smach state
    """
    def test_drive(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_DRIVE
        origin_node = Node(SOURCE_NODE_ID, SOURCE_NODE_AREA)
        msg.origin = origin_node
        destination_node = Node(DESTINATION_NODE_ID, DESTINATION_NODE_AREA)
        msg.destination = destination_node
        actions = convert_msgs_to_actions(ROBOT, [msg])
        self.assertEquals(len(actions), 1, "Result should contain exactly one action")
        self.assertTrue(isinstance(actions[0], NavigateToSymbolic), "Action should be a 'NavigateToSymbolic' action")

    def test_drive_waypoint(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_DRIVE
        origin_node = Node(SOURCE_NODE_ID, SOURCE_NODE_AREA)
        msg.origin = origin_node
        destination_node = Node()
        destination_node.entity = DESTINATION_NODE_ID
        msg.destination = destination_node
        actions = convert_msgs_to_actions(ROBOT, [msg])
        self.assertEquals(len(actions), 1, "Result should contain exactly one action")
        self.assertTrue(isinstance(actions[0], NavigateToWaypoint), "Action should be a 'NavigateToWaypoint' action")

    def test_open_door(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_OPEN_DOOR
        origin_node = Node(SOURCE_NODE_ID, SOURCE_NODE_AREA)
        msg.origin = origin_node
        destination_node = Node()
        destination_node.entity = DESTINATION_NODE_ID
        msg.destination = destination_node
        actions = convert_msgs_to_actions(ROBOT, [msg])
        self.assertEquals(len(actions), 1, "Result should contain exactly one action")
        self.assertTrue(isinstance(actions[0], OpenDoor), "Action should be a 'OpenDoor' action")

    def test_push_object(self):
        msg = Edge()
        msg.action_type = Edge.ACTION_PUSH_OBJECT
        origin_node = Node(SOURCE_NODE_ID, SOURCE_NODE_AREA)
        msg.origin = origin_node
        destination_node = Node()
        destination_node.entity = DESTINATION_NODE_ID
        msg.destination = destination_node
        actions = convert_msgs_to_actions(ROBOT, [msg])
        self.assertEquals(len(actions), 1, "Result should contain exactly one action")
        self.assertTrue(isinstance(actions[0], PushObject), "Action should be a 'PushObject' action")

    def test_invalid_action(self):
        msg = Edge()
        msg.action_type = "foo"
        self.assertRaises(TopologicalPlannerException, convert_msgs_to_actions, ROBOT, [msg])
