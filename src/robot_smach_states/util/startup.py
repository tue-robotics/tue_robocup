"""
State machine startup

Usage:
  challenge_{challenge_name}.py {robot}
  challenge_{challenge_name}.py {robot} --initial=<init>

Options:
  -h --help     Show this screen.
  --initial=<init>  Initial state
"""


import rospy
import smach_ros
import sys
import traceback
from docopt import docopt
import os


def startup(statemachine_creator, initial_state=None, robot_name='', challenge_name=None):
    '''
    :param statemachine_creator: a function that outputs a statemachine.
        The function should take a robot as input.
    :param initial_state the state to start the state machine in. 
        Can be supplied as command line argument
    :param robot_name name of the robot to pass to the state machine'''

    if initial_state or robot_name:
        rospy.logwarn(  "Setting initial_state and robot_name via the startup"
                        " is not needed and deprecated. "
                        "This is inferred by startup from the command line")

    available_robots = ['amigo', 'sergio', 'mockbot']
    arguments = docopt(__doc__.format(robot='|'.join(available_robots),
                                      challenge_name=challenge_name if challenge_name else "xxx"),
                       version='robot_smach_states startup 2.0')
    robot_name = [robotname for robotname in available_robots if arguments[robotname] ][0]
    initial_state = arguments["--initial"]

    robot = None
    # if robot_name == "amigo":
    #     import robot_skills.amigo
    #     robot = robot_skills.amigo.Amigo(wait_services=True)
    # elif robot_name == "sergio":
    #     import robot_skills.sergio
    #     robot = robot_skills.sergio.Sergio(wait_services=True)
    # elif robot_name == "mockbot":
    import robot_skills.mockbot
    # import ipdb; ipdb.set_trace()
    robot = robot_skills.mockbot.Mockbot(wait_services=True)

    # else:
    #     rospy.logerr(
    #         "No robot named '{}'. Options: {}"\
    #             .format(robot_name, available_robots))
    #     exit(-1)

    rospy.loginfo("Using robot '" + robot_name + "'.")

    introserver = None

    with robot:
        try:
            # build the state machine
            executioner = None #statemachine_creator(robot)
            if initial_state:
                initial_state = [initial_state]
                rospy.logwarn(
                    "Overriding initial state with {}".format(initial_state))
                executioner.set_initial_state(initial_state)

            # introserver = smach_ros.IntrospectionServer(
            #     statemachine_creator.__name__, executioner, '/SM_ROOT_PRIMARY')
            # introserver.start()

            # Run the statemachine
            # outcome = executioner.execute()
            outcome = None
            print "Final outcome: {0}".format(outcome)
        except Exception, e:
            print "An exception occured"
            frame = traceback.extract_tb(sys.exc_info()[2])[0]
            fname, lineno, fn, text = frame
            rospy.logerr(
                "Error: {0},{1},{2},{3}".format(fname, lineno, fn, text))

            fname_stripped = fname.split("/")[-1:][0]

            message = "I encountered an error in '{0}'' on line {1}: '{4}'. " \
                      "Can I get a restart and try again?"\
                .format(fname_stripped, lineno, fn, text, e)
            message = message.replace("_", " ")
            message = message.replace(".py", "")

            print[fname, lineno, fn, text, e]
#            robot.speech.speak(message)
        finally:
            if introserver:
                print "Stopping introserver"
                introserver.stop()
                print "Introserver stopped"
                del introserver

            # list_threads()
            # kill_proc_tree(os.getpid()) #Sometimes scripts won't terminate nicely, so we kill it the hard way.

def kill_proc_tree(pid, including_parent=True):    
    import psutil
    parent = psutil.Process(pid)
    try:
        if parent.is_running():
            parent.wait(5)
    except psutil.TimeoutExpired:
        print "Parent process won't die nicely, so kill the children first. Sounds harsh, but it just a process"
        children = []
        try:
            children = parent.get_children()
        except AttributeError:
            children = parent.children()
        
        for child in children:
            child.kill()
        for child in children:
            child.wait(timeout=5)
        if including_parent:
            parent.kill()
            parent.wait(5)

def list_threads():
    print >> sys.stderr, "\n*** STACKTRACE - START ***\n"
    code = set()
    for threadId, stack in sys._current_frames().items():
        thread_description = "\n# ThreadID: "
        for filename, lineno, name, line in traceback.extract_stack(stack):
            thread_description += 'File: "%s", line %d, in %s\n' % (filename, lineno, name)
            if line:
                thread_description += "  %s\n" % (line.strip())
        code.add(thread_description)

    for line in code:
        print >> sys.stderr, line
    print >> sys.stderr, "\n*** STACKTRACE - END ***\n"