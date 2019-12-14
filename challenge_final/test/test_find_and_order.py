import argparse
import rospy
import smach

from challenge_final import FindPeople, GetOrders
from robot_skills import get_robot


class TestMachine(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:

            smach.StateMachine.add("FIND_PEOPLE", FindPeople(robot), transitions={"done": "GET_ORDERS"})
            smach.StateMachine.add("GET_ORDERS", GetOrders(robot), transitions={"done": "done"})


if __name__ == "__main__":

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--robot", default="hero", help="Name of the robot you want to use")
    args = parser.parse_args()

    rospy.init_node("test_find_and_order")

    robot = get_robot(args.robot)

    sm = TestMachine(robot)
    sm.execute()
