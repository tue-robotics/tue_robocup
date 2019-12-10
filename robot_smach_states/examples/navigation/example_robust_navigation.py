# ROS
import rospy

# TU/e Robotics
from robot_skills import get_robot_from_argv

# Robot smach states
from robot_smach_states.navigation import NavigateRobust, NavigateToWaypoint
import robot_smach_states.util.designators as ds


if __name__ == "__main__":

    # Initialize rosnode
    rospy.init_node("test_robust_navigation")

    # Construct robot object
    robot = get_robot_from_argv(index=1)

    # Construct statemachine
    sm = NavigateRobust(
        robot,
        options=[
            NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id="foo")),
            NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id="bar")),
            NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id="initial_pose"))
        ],
        wait_time=1.0)

    sm.execute()
