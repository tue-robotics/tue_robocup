# System
import mock
import unittest

# TU/e Robotics
from topological_action_planner.msg import Edge, Node

# Robot Smach States
from robot_smach_states.navigation.navigation import NavigateTo
from robot_smach_states.navigation.topological_navigation import convert_msgs_to_actions

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
        msg.u = origin_node
        destination_node = Node(DESTINATION_NODE_ID, DESTINATION_NODE_AREA)
        msg.v = destination_node
        actions = convert_msgs_to_actions(ROBOT, [msg])
        self.assertEquals(len(actions), 1, "Result should contain exactly one action")
        self.assertTrue(isinstance(actions[0], NavigateTo), "Action should be a 'NavigateTo' action")

    @unittest.skip("Not yet implemented")
    def test_drive_waypoint(self):
        pass

    def test_open_door(self):
        pass

    def test_push_object(self):
        pass

# Actions: [u:
#   entity: ''
#   area: ''
# v:
#   entity: "inner_door"
#   area: "in_front_of"
# action_type: 1
# cost: 1.0, u:
#   entity: "inner_door"
#   area: "in_front_of"
# v:
#   entity: "inner_door"
#   area: "in_back_of"
# action_type: 2
# cost: 10.0, u:
#   entity: "inner_door"
#   area: "in_back_of"
# v:
#   entity: ''
#   area: ''
# action_type: 1
# cost: 1.0]
