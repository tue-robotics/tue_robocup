"""
State machine startup

Usage:
  challenge_{challenge_name}.py ({robot}) [--initial=<init>] [--initial_pose=<init_pose>] [--debug] [--no-execute]

Options:
  -h --help                     Show this screen.
  --initial=<init>              Initial state
  --initial_pose=<init_pose>    Initial state
  --debug                       Run the IntrospectionServer
  --no-execute                  Only construct state machine, do not execute it, i.e. only do checks.
"""

# System
import ast
from docopt import docopt
import sys
import time

# ROS
import rospy
import smach_ros

# TU/e Robotics
from robot_skills.util.robot_constructor import robot_constructor


def startup(statemachine_creator, statemachine_args = (), initial_state=None, robot_name='', challenge_name=None, argv=sys.argv):
    """
    :param statemachine_creator: a function that outputs a statemachine.
        The function should take a robot as its first input.
    :param statemachine_args: A list of arguments. If the statemachine_creator
        function takes any arguments besides robot these can be placed here.
    :param initial_state: the state to start the state machine in.
        Can be supplied as command line argument
    :param robot_name: name of the robot to pass to the state machine
    """
    t_start = time.time()
    if initial_state or robot_name:
        rospy.logwarn("Setting initial_state and robot_name via the startup"
                      "is not needed and deprecated. "
                      "This is inferred by startup from the command line")

    available_robots = ['amigo', 'sergio', 'hero', 'mockbot']
    arguments = docopt(__doc__.format(robot='|'.join(available_robots),
                                      challenge_name=challenge_name if challenge_name else "xxx"),
                       argv=[v for v in argv[1:] if not v.startswith("_")],
                       version='robot_smach_states startup 2.0')
    robot_name = [robotname for robotname in available_robots if arguments[robotname] ][0]
    initial_state = arguments["--initial"]
    initial_pose = arguments["--initial_pose"]
    enable_debug = arguments["--debug"]
    no_execute = arguments["--no-execute"]

    robot = robot_constructor(robot_name)

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
