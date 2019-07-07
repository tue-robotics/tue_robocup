# ROS
import smach

# TU/e Robotics
import robot_smach_states as states

# Challenge final
from .find_people import FindPeople
from .get_drinks import GetDrinks
from .get_orders import GetOrders
from .lightsaber import DriveAndSwordFight, LightSaber


class Final(smach.StateMachine):
    def __init__(self, robot):
        """
        Final challenge of RWC 2019 Sydney

        :param robot: (Robot) api object
        """
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:

            # smach.StateMachine.add("START_ROBUST",
            #                        states.StartChallengeRobust(robot, "initial_pose"),
            #                        transitions={"Done": "LASER_POINTING",
            #                                     "Aborted": "LASER_POINTING",
            #                                     "Failed": "LASER_POINTING"})

            smach.StateMachine.add("INIT", states.Initialize(robot),
                                   transitions={"initialized": "LASER_POINTING",
                                                "abort": "LASER_POINTING"})

            smach.StateMachine.add("LASER_POINTING",
                                   DriveAndSwordFight(robot),
                                   transitions={"done": "FIND_PEOPLE"})

            smach.StateMachine.add("FIND_PEOPLE",
                                   FindPeople(robot),
                                   transitions={"done": "GET_ORDERS"})

            smach.StateMachine.add("GET_ORDERS",
                                   GetOrders(robot),
                                   transitions={"done": "GET_DRINKS"})

            smach.StateMachine.add("GET_DRINKS",
                                   GetDrinks(robot=robot),
                                   transitions={"done": "done",
                                                "failed": "done"})

            smach.StateMachine.add("SHOW_PEOPLE_DETECTION",
                                   LightSaber(robot),
                                   transitions={"done": "done"})
