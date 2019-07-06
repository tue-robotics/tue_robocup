# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
from robot_skills import get_robot_from_argv


class GetOrders(smach.StateMachine):
    def __init__(self, robot):
        """
        Gets the orders of the detected people using the Telegram interface.

        * Telegram interaction
        * Drive to operator
        * Display order on screen

        Input keys:
        * detected_people: a list of {'rgb':..., 'person_detection':..., 'map_ps':...}-dictionaries

        Output keys:
        * orders: ToDo: Loy: define this

        :param robot: (Robot) api object
        """
        smach.StateMachine.__init__(self,
                                    outcomes=["done"],
                                    input_keys=["detected_people"],
                                    output_keys=["orders"])

        with self:

            smach.StateMachine.add("SAY_YEEHAH",
                                   states.Say(robot, "Loy will make sure I will do something useful here"),
                                   transitions={"spoken": "done"})

            # Here comes Loys stuff (stuff is passed)

            # Fuse people and orders  # ToDo: Janno


if __name__ == "__main__":

    rospy.init_node("test_furniture_inspection")

    # Robot
    _robot = get_robot_from_argv(index=1)

    # Test data
    # ToDo: load Reins pickled file here
    user_data = smach.UserData()

    sm = GetOrders(robot=_robot)
    sm.execute(user_data)
