# ROS
import smach

# TU/e Robotics
import robot_smach_states as states

# Challenge final
from .find_people import FindPeople
from .get_orders import GetOrders


class Final(smach.StateMachine):
    def __init__(self, robot):
        """
        Final challenge of RWC 2019 Sydney

        :param robot: (Robot) api object
        """
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:

            # ToDo: awesome people detection stuff

            smach.StateMachine.add("FIND_PEOPLE",
                                   FindPeople(robot),
                                   transitions={"done": "GET_ORDERS"})

            smach.StateMachine.add("GET_ORDERS",
                                   GetOrders(robot),
                                   transitions={"done": "done"})
