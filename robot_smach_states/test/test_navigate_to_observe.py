# ROS
import rospy

# TU/e Robotics
from robot_skills.mockbot import Mockbot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states.navigation import NavigateToObserve


if __name__ == "__main__":

    rospy.init_node("test_navigate_to_observe")

    robot = Mockbot()

    entity = ds.EdEntityDesignator(robot, id="trashbin")

    observe_state = NavigateToObserve(robot, entity)

    observe_state.execute()

