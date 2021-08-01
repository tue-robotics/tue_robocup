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

    def get_plan(self, entity_id: str, area: str = None) -> List[Edge]:  # ToDo: add request argument
        """
        Gets a plan from the topological action server

        :param entity_id: string identifying the entity
        :param area: string identifying the area. If this is not defined, the result may be, e.g., a waypoint
        :return: a list of actions to take
        """
        request = PlanRequest()
        request.origin = Node("", "")  # ToDo: remove!!!
        request.destination = Node(entity_id, area)
        response = self._planner_srv(request)
        return response.edges

    def collapse_plan(self, plan: List[Edge]) -> List[Edge]:
        """
        If a plan consists of just driving, there is no need to travel along all of the waypoints
        Only if there is an action like opening a door that we really
            need to go to the place from where a door needs to be opened, for example.
        If consecutive edges can all be driven via, we might as well use normal navigation.
        """
        simple = []
        for edge in plan:
            if simple and simple[-1].action_type == Edge.ACTION_DRIVE == edge.action_type:
                simple[-1][1] = edge[1]
            else:
                simple += [list(edge)]
        return simple

