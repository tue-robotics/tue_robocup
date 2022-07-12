# System
from typing import List

# ROS
import tf2_ros

# TU/e Robotics
from topological_action_planner_msgs.srv import Plan, PlanRequest, PlanResponse
from topological_action_planner_msgs.msg import Edge, Node

# Robot skills
from .robot_part import RobotPart


class TopologicalPlanner(RobotPart):
    def __init__(self, robot_name: str, tf_buffer: tf2_ros.Buffer):
        """
        Interface to the topological planner. The topological action planner determines a list of actions, e.g.,
        drive, open door, push object, that the robot must perform in order to get to a destination node.

        :param robot_name: name of the robot (for namespacing)
        :param tf_buffer: tf buffer
        """
        super(TopologicalPlanner, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)

        # Topological action planner
        self._planner_srv = rospy.ServiceProxy(
            "/" + self.robot_name + "/topological_action_planner/get_plan",
            Plan,
        )

    def get_plan(self, entity_id: str, area: str = None) -> List[Edge]:
        """
        Gets a plan from the topological action server

        :param entity_id: string identifying the entity
        :param area: string identifying the area. If this is not defined, the result may be, e.g., a waypoint
        :return: a list of actions to take
        """
        request = PlanRequest()
        request.destination = Node(entity_id, area)
        response = self._planner_srv(request)  # type: PlanResponse
        return response.edges
