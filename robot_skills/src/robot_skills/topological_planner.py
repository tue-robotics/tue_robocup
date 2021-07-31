# System
import typing

# ROS
import tf2_ros

# TU/e Robotics
from topological_action_planner.srv import Plan, PlanRequest, PlanResponse
from topological_action_planner.msg import Edge

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

    def get_plan(self) -> typing.List[Edge]:  # ToDo: add request argument
        """
        Gets a plan from the topological action server

        :return: a list of actions to take
        """
        request = PlanRequest()
        response = self._planner_srv(request)
        return response.edges
