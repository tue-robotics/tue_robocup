#! /usr/bin/env python

# ROS
import rospy
import smach

# TU/e Robotics
import hmi
import robot_smach_states.util.designators as ds

from robot_smach_states import Initialize, Say
from robot_smach_states.util.startup import startup


def fetch(robot, time_out=15.0, task="set"):
    """

    :param robot: robot object
    :param time_out: timeout in seconds (not yet used)
    :param task: string with set or clear
    :return: string with heard or not heard
    """

    confirmed = False
    robot.speech.speak('What can I do for you, master?')

    # Listen for the new task
    i = 0
    while not confirmed and i < 10:
        try:
            if task == "set":
                sentence, semantics = robot.hmi.query('', 'T -> set the table', 'T')
                return "heard"
            elif task == "clear":
                sentence, semantics = robot.hmi.query('', 'T -> set the table', 'T')
                return "heard"
            else:
                rospy.logerr("This does not make any sense!!!")

            # check if we have heard this correctly
            robot.speech.speak('I heard %s, is this correct?' % sentence)
            try:
                result = robot.hmi.query('', 'T -> yes | no', 'T').sentence
                if result == 'yes':
                    confirmed = True
                elif result == 'no':
                    robot.speech.speak('Sorry, please repeat')
                    pass

            except hmi.TimeoutException:
                confirmed = True
                # robot did not hear the confirmation, so lets assume its correct

        except hmi.TimeoutException:
            i += 1

    return "not_heard"


class HearFetchCommand(smach.State):
    def __init__(self, robot, time_out=15.0, task="set"):
        """ Constructor

        :param robot: robot object
        :param time_out: timeout in seconds (not yet used)
        :param task: string with set or clear
        """
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.time_out = time_out
        if task not in ["set", "clear"]:
            rospy.logerr("You either have to supply the set or clear task")
        self.task = task

    def execute(self, userdata):

        self.robot.head.look_at_standing_person()

        result = fetch(self.robot, time_out=self.time_out)
        if result != "heard":
            self.robot.speech.speak("I am sorry but I did not hear you. I will start doing some work anyway")

        self.robot.head.reset()

        return "done"


class GetBreakfastOrder(smach.State):
    """ Gets the breakfast order by asking it to the customer. N.B.: the order is asked, but this information is not
     stored (we won't get points for that anyway """
    def __init__(self, robot, options, timeout=15.0):
        """ Constructor

        :param robot: robot object
        :param options: list with strings containing the possible breakfast choices
        :param timeout: timeout in seconds
        """
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.options = options
        self.grammar = "T[O] -> SENTENCE[O]\n\n"
        self.grammar += "DET -> the | a | an | some\n\n"
        self.grammar += "SENTENCE[O] -> could you bring me DET OPTIONS[O]\n"
        self.grammar += "SENTENCE[O] -> can you bring me DET OPTIONS[O]\n"
        self.grammar += "SENTENCE[O] -> could you give me DET OPTIONS[O]\n"
        self.grammar += "SENTENCE[O] -> can you give me DET OPTIONS[O]\n"
        self.grammar += "SENTENCE[O] -> please bring me DET OPTIONS[O]\n"
        self.grammar += "SENTENCE[O] -> bring me DET OPTIONS[O]\n"
        self.grammar += "SENTENCE[O] -> i would like DET OPTIONS[O]\n"
        self.grammar += "SENTENCE[O] -> i want DET OPTIONS[O]\n\n"
        for option in options:
            self.grammar += "OPTIONS['{}'] -> {}\n".format(option, option)

        print self.grammar
        self.timeout = timeout

    def execute(self, userdata):

        self.robot.speech.speak("What would you like to have for breakfast?", block=False)

        self.robot.head.look_at_standing_person()

        # Try for a max of 3 times
        i = 0
        while i < 3:

            i += 1

            try:
                sentence, semantics = self.robot.hmi.query('', self.grammar, 'T')  # ToDo: add timeout
                order = semantics

                # check if we have heard this correctly
                self.robot.speech.speak('I understood you would like {}, is this correct?'.format(order))
                try:
                    result = self.robot.hmi.query('', 'T -> yes | no', 'T').sentence
                    if result == 'yes':
                        self.robot.speech.speak("Okay, I will bring you {}".format(order))
                        return "done"
                    elif result == 'no':
                        self.robot.speech.speak('Sorry, please repeat')
                        pass

                except hmi.TimeoutException:
                    return "done"
                    # robot did not hear the confirmation, so lets assume its correct

            except hmi.TimeoutException:
                pass

        # If nothing has been heard, make a guess
        self.robot.speech.speak("I will bring you {}".format(self.options[0]))

        return "done"


# Standalone testing -----------------------------------------------------------------~
class TestFetchCommand(smach.StateMachine):
        def __init__(self, robot):
                smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

                with self:
                        smach.StateMachine.add('INITIALIZE',
                                               Initialize(robot),
                                               transitions={'initialized': 'FETCH_COMMAND',
                                                            'abort': 'Aborted'})

                        smach.StateMachine.add('FETCH_COMMAND',  # Hear "set the table"
                                               HearFetchCommand(robot, 15.0),
                                               transitions={'heard': 'END_CHALLENGE'})

                        # End
                        smach.StateMachine.add('END_CHALLENGE',
                                               Say(robot, "I am finally free!"),
                                               transitions={'spoken': 'Done'})

                        ds.analyse_designators(self, "set_a_table")


if __name__ == "__main__":
        rospy.init_node('set_a_table_exec')

        startup(TestFetchCommand, challenge_name="challenge_set_a_table")
