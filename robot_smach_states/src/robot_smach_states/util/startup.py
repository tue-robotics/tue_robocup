"""
State machine startup

Usage:
  {challenge_name} ({robot}) [--initial=<init>] [--initial_pose=<init_pose>] [--debug] [--no-execute] [-t TIMEOUT]

Options:
  -h --help                                Show this screen.
  --initial=<init>                         Initial state
  --initial_pose=<init_pose>               Initial state
  --debug                                  Run the IntrospectionServer
  --no-execute                             Only construct state machine, do not execute it, i.e. only do checks.
  -t TIMEOUT --connection-timeout=TIMEOUT  Timeout for connecting ROS connections [default: {default_timeout}]
"""

from __future__ import absolute_import
import __main__

# System
import ast
from docopt import docopt
import os.path
import sys
import time

# ROS
import rospy
import smach_ros

# TU/e Robotics
from robot_skills.get_robot import get_robot, ROBOTS
from robot_skills.robot import DEFAULT_CONNECTION_TIMEOUT


def startup(
    statemachine_creator, statemachine_args=None, challenge_name=None, argv=None
):
    """
    :param statemachine_creator: a function that outputs a statemachine.
        The function should take a robot as its first input.
    :param statemachine_args: A list of arguments. If the statemachine_creator
        function takes any arguments besides robot these can be placed here.
    :param challenge_name: name of the challenge
    :type challenge_name: str
    :param argv: argument values, stripped from ros arguments (default: rospy.myargv())
    :type argv: list
    """
    t_start = time.time()
    if statemachine_args is None:
        statemachine_args = []
    if argv is None:
        argv = rospy.myargv()

    available_robots = ROBOTS.keys()
    try:
        arguments = docopt(
            __doc__.format(
                robot="|".join(available_robots),
                challenge_name=challenge_name if challenge_name else os.path.basename(__main__.__file__),
                default_timeout=DEFAULT_CONNECTION_TIMEOUT,
            ),
            argv=argv[1:],
            version="robot_smach_states startup 2.0",
        )
    except SystemExit as e:  # In this except we act as it is a DocoptExit
        rospy.logfatal(f"Invalid command-line arguments provided: {argv}\nDocopt could not match it against:\n{e.usage}")
        sys.exit(1)

    robot_name = [robotname for robotname in available_robots if arguments[robotname]][0]
    initial_state = arguments["--initial"]
    initial_pose = arguments["--initial_pose"]
    enable_debug = arguments["--debug"]
    no_execute = arguments["--no-execute"]
    connection_timeout = arguments["--connection-timeout"]

    robot = get_robot(robot_name, connection_timeout=connection_timeout)

    rospy.loginfo("Using robot '" + robot_name + "'.")

    if initial_pose:
        # Parse the initial pose array
        try:
            x, y, yaw = ast.literal_eval(initial_pose)

            rospy.loginfo("Setting initial pose of (%f,%f,%f)", x, y, yaw)
            robot.base.set_initial_pose(x, y, yaw)
        except Exception:
            rospy.logerr("Failed to parse initial pose x, y, yaw")

    introserver = None

    with robot:
        # build the state machine
        executioner = statemachine_creator(robot, *statemachine_args)
        if initial_state:
            initial_state = [initial_state]
            rospy.logwarn(
                "Overriding initial state with {}".format(initial_state))
            executioner.set_initial_state(initial_state)

        if enable_debug:
            introserver = smach_ros.IntrospectionServer(robot_name, executioner, '/SM_ROOT_PRIMARY')
            introserver.start()

        if not no_execute:
            # Run the statemachine
            outcome = executioner.execute()
            rospy.loginfo("Final outcome: {0}".format(outcome))

        if introserver:
            introserver.stop()

    t_end = time.time()
    duration = t_end - t_start
    rospy.loginfo("Execution of {} took {} minutes and {} seconds".format(
        executioner.__class__.__name__,
        int(duration // 60),
        int(duration % 60),
    ))
