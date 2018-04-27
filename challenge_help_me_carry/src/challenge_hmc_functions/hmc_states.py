#!/usr/bin/python

import smach
import robot_smach_states as states

from robot_smach_states.util.designators import check_type
from robot_skills.arms import Arm, GripperState
from hmi import TimeoutException

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_help_me_carry')


class WaitForOperatorCommand(smach.State):
    """
    The robot waits for command from the operator
    Possible commands are two types:
        - commands as outcomes: each possible command is possible outcome
        - commands as userdata: the command is passed to the next state

    """
    def __init__(self, robot, possible_commands, commands_as_outcomes=False, commands_as_userdata=False, target=None):

        self._robot = robot
        self._possible_commands = possible_commands
        self._commands_as_outcomes = commands_as_outcomes
        self._commands_as_userdata = commands_as_userdata
        self.target_destination = target

        if commands_as_outcomes:  # each possible command is a separate outcome
            _outcomes = possible_commands + ['abort']
        else:  # outcome is success or abort, recognized command is returned using output_keys
            _outcomes = ['success', 'abort']

        if commands_as_userdata:  # pass the recognized command to the next state
            _output_keys = ['command_recognized']
        else:  # do not pass data to the next state
            _output_keys = []

        smach.State.__init__(self, outcomes=_outcomes, output_keys=_output_keys)

    def _listen_for_commands(self, tries=5, time_out=30):
        for i in range(0, tries):
            try:
                result = self._robot.hmi.query('What command?', 'T -> ' + ' | '.join(self._possible_commands), 'T',
                                               timeout=time_out)
                command_recognized = result.sentence
            except TimeoutException:
                command_recognized = None
            if command_recognized == "":
                self._robot.speech.speak("I am still waiting for a command and did not hear anything")

            elif command_recognized in self._possible_commands:
                if command_recognized == "no":
                    self._robot.speech.speak("OK")
                else:
                    self._robot.speech.speak("OK, I will go to {}".format(command_recognized))
                return "success", command_recognized
            else:
                self._robot.speech.speak(
                    "I don't understand, I expected a command like " + ", ".join(self._possible_commands))

        self._robot.speech.speak("I did not recognize a command and will stop now")
        return "abort", "abort"

    def execute(self, userdata):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()
        self._robot.head.look_at_standing_person()

        outcome, command_recognized = self._listen_for_commands()

        if self._commands_as_userdata:
            userdata.command_recognized = command_recognized
            self.target_destination.id_ = command_recognized

        if self._commands_as_outcomes:
            return command_recognized
        else:
            return outcome


class StoreCarWaypoint(smach.State):
    """
    The robot remembers the position of the car for future use

    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success', 'abort'])
        self._robot = robot

    def execute(self, userdata=None):
        response = self._robot.ed.update_entity(id=challenge_knowledge.waypoint_car['id'],
                                               frame_stamped=self._robot.base.get_location(),
                                               type="waypoint")

        if not response.response:
            return "success"
        else:
            return "abort"


class DropBagOnGround(smach.StateMachine):
    """
    Put the bag in the robot's gripper on the ground

    """
    def __init__(self, robot, arm_designator, drop_bag_pose):
        """
        :param robot: the robot with which to execute this state machine
        :param arm_designator: ArmDesignator resolving to Arm holding the bag to drop

        """
        smach.StateMachine.__init__(self, outcomes=['done'])

        check_type(arm_designator, Arm)

        with self:
            smach.StateMachine.add('DROP_POSE', states.ArmToJointConfig(robot, arm_designator, drop_bag_pose),
                                   transitions={'succeeded': 'OPEN_AFTER_DROP',
                                                'failed': 'OPEN_AFTER_DROP'})

            smach.StateMachine.add('OPEN_AFTER_DROP',
                                   states.SetGripper(robot, arm_designator, gripperstate=GripperState.OPEN),
                                   transitions={'succeeded': 'RESET_ARM',
                                                'failed': 'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM', states.ResetArms(robot),
                                   transitions={'done': 'done'})


class LearnOperator(smach.State):
    def __init__(self, robot, operator_timeout=20, ask_follow=True, learn_face=True, learn_person_timeout=10.0):
        smach.State.__init__(self, outcomes=['learned', 'failed'])
        self._robot = robot
        self._operator_timeout = operator_timeout
        self._ask_follow = ask_follow
        self._learn_face = learn_face
        self._learn_person_timeout = learn_person_timeout
        self._operator_name = "operator"
        self._operator = None

    def execute(self, userdata):
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("I will memorize what you look like now.",
                                 block=True)
        start_time = rospy.Time.now()
        self._robot.head.look_at_standing_person()
        while not self._operator:
            if self.preempt_requested():
                return 'failed'

            if (rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return 'failed'

            self._operator = self._robot.ed.get_closest_laser_entity(
                radius=0.5,
                center_point=kdl_conversions.VectorStamped(x=1.0, y=0, z=1,
                                                           frame_id="/%s/base_link" % self._robot.robot_name))
            rospy.loginfo("Operator: {op}".format(op=self._operator))
            if not self._operator:
                self._robot.speech.speak("Please stand in front of me", block=True)
            else:
                if self._learn_face:
                    self._robot.speech.speak("Please look at me while I learn to recognize you.",
                                             block=True)
                    self._robot.head.look_at_standing_person()
                    learn_person_start_time = rospy.Time.now()
                    num_detections = 0
                    while num_detections < 5:  # 5:
                        if self._robot.perception.learn_person(self._operator_name):
                            print("Successfully detected you %i times" % (num_detections + 1))
                            num_detections += 1
                        elif (rospy.Time.now() - learn_person_start_time).to_sec() > self._learn_person_timeout:
                            self._robot.speech.speak("Please stand in front of me and look at me")
                            self._operator = None
                            break
        print "We have a new operator: %s" % self._operator.id
        self._robot.speech.speak("Gotcha! Please follow me to the car.", block=False)
        self._robot.head.close()
        return 'learned'
