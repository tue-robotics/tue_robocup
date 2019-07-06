# ROS
import smach

# TU/e Robotics
import robot_smach_states as states


class Final(smach.StateMachine):
    def __init__(self, robot):
        """
        Final challenge of RWC 2019 Sydney

        :param robot: (Robot) api object
        """
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:

            smach.StateMachine.add("SAY_YEEHAH",
                                   states.Say(robot, "Yeehah"),
                                   transitions={"spoken": "done"})
