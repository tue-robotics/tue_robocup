# System
import typing

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
        Interface to the topological planner.

        :param robot_name:
        :param tf_buffer:
        """
        super(TopologicalPlanner, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)

        # Topological action planner
        self._planner_srv = self.create_service_client(
            "/" + self.robot_name + "/topological_action_planner/get_plan",
            Plan,
        )

<<<<<<< HEAD
    def get_plan(self) -> typing.List[Edge]:  # ToDo: add request argument
        """
        Gets a plan from the topological action server

        :return: a list of actions to take
        """
        request = PlanRequest()
=======
    def get_plan(self, entity_id: str, area: str = None) -> typing.List[Edge]:  # ToDo: add request argument
        """
        Gets a plan from the topological action server

        :param entity_id: string identifying the entity
        :param area: string identifying the area. If this is not defined, the result may be, e.g., a waypoint
        :return: a list of actions to take
        """
        request = PlanRequest()
        request.origin = Node("bar", area)  # ToDo: remove!!!
        request.destination = Node(entity_id, area)
>>>>>>> biestheuvel
        response = self._planner_srv(request)
        return response.edges
