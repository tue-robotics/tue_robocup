#!/usr/bin/python
import roslib;
import rospy
import smach
import sys

import robot_smach_states as states
from robot_smach_states.util.designators.designator import Designator, EdEntityDesignator

import data

class HearQuestion(smach.State):
    def __init__(self, robot, spec, choices, time_out=rospy.Duration(10)):
        smach.State.__init__(self, outcomes=["heard", "not_heard"])
        self.robot = robot
        self.spec = spec
        self.choices = choices
        self.time_out = time_out

    def execute(self, userdata):

        res = self.robot.ears.recognize(spec=self.spec, choices=self.choices, time_out=self.time_out)

        if not res:
            self.robot.speech.speak("I could not hear your question.")
            return "not_heard"

        try:
            rospy.loginfo("Question was: '{0}'?".format(res.result))
            self.robot.speech.speak("The answer is '{0}'".format(data.answers[list(res.choices.keys())[0]][0]))

        except KeyError:
            rospy.logwarn("Amigo did not understand user")
            return "not_heard"

        return "heard"


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE_ROBUST",
                                states.StartChallengeRobust(robot, "initial_pose", use_entry_points = True),
                                transitions={   "Done"              :   "SAY_GOTO_INTERROGATION_POINT",
                                                "Aborted"           :   "SAY_GOTO_INTERROGATION_POINT",
                                                "Failed"            :   "SAY_GOTO_INTERROGATION_POINT"})  

        smach.StateMachine.add( 'SAY_GOTO_INTERROGATION_POINT',
                                states.Say(robot, ["I will go to the interrogation point now",
                                                    "I will now go to the interrogation point",
                                                    "Lets go to the interrogation point",
                                                    "Going to the interrogation point"], block=False),
                                #transitions={   'spoken'            :   'GOTO_INTERROGATION_POINT_1'})
                                transitions={   'spoken'            :   'HEAR_QUESTION_1'})


        smach.StateMachine.add('GOTO_INTERROGATION_POINT_1',
                                states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="interrogation_point_1", parse=True, radius=0.7)),
                                transitions={   'arrived'           :   'SAY_INTERROGATION_POINT_REACHED',
                                                'unreachable'       :   'GOTO_INTERROGATION_POINT_2',
                                                'goal_not_defined'  :   'GOTO_INTERROGATION_POINT_2'})

        smach.StateMachine.add('GOTO_INTERROGATION_POINT_2',
                                states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="interrogation_point_2", parse=True, radius=0.7)),
                                transitions={   'arrived'           :   'SAY_INTERROGATION_POINT_REACHED',
                                                'unreachable'       :   'SAY_SAY_INTERROGATION_POINT_REACHED_FAILED',
                                                'goal_not_defined'  :   'SAY_SAY_INTERROGATION_POINT_REACHED_FAILED'})

        smach.StateMachine.add( 'SAY_INTERROGATION_POINT_REACHED',
                                states.Say(robot, ["What can I do for you?",
                                                    "You had a question?"], block=True),
                                transitions={   'spoken'            :   'HEAR_QUESTION_1'})       

        smach.StateMachine.add( 'SAY_SAY_INTERROGATION_POINT_REACHED_FAILED',
                                states.Say(robot, ["I am not able to reach the interrogation point, what can I do for you?",
                                                    "I am not able to reach the interrogation point, you had a question"], block=True),
                                transitions={   'spoken'            :   'HEAR_QUESTION_1'})

        smach.StateMachine.add( 'HEAR_QUESTION_1',
                                HearQuestion(robot, spec=data.spec, choices=data.choices),
                                transitions={   'heard'            :   'SAY_NEXT_QUESTION_1',
                                                'not_heard'        :   'SAY_NEXT_QUESTION_1'}) 

        smach.StateMachine.add( 'SAY_NEXT_QUESTION_1',
                                states.Say(robot, ["What is your next question?"], block=True),
                                #transitions={   'spoken'            :   'GO_TO_EXIT'})
                                transitions={   'spoken'            :   'HEAR_QUESTION_2'})

        smach.StateMachine.add( 'HEAR_QUESTION_2',
                                HearQuestion(robot, spec=data.spec, choices=data.choices),
                                transitions={   'heard'            :   'SAY_NEXT_QUESTION_2',
                                                'not_heard'        :   'SAY_NEXT_QUESTION_2'}) 

        smach.StateMachine.add( 'SAY_NEXT_QUESTION_2',
                                states.Say(robot, ["What is your next question?"], block=True),
                                #transitions={   'spoken'            :   'GO_TO_EXIT'})
                                transitions={   'spoken'            :   'HEAR_QUESTION_3'})

        smach.StateMachine.add( 'HEAR_QUESTION_3',
                                HearQuestion(robot, spec=data.spec, choices=data.choices),
                                transitions={   'heard'            :   'SAY_NEXT_QUESTION_3',
                                                'not_heard'        :   'SAY_NEXT_QUESTION_3'}) 

        smach.StateMachine.add( 'SAY_NEXT_QUESTION_3',
                                states.Say(robot, ["What is your next question?"], block=True),
                                #transitions={   'spoken'            :   'GO_TO_EXIT'})
                                transitions={   'spoken'            :   'HEAR_QUESTION_4'})

        smach.StateMachine.add( 'HEAR_QUESTION_4',
                                HearQuestion(robot, spec=data.spec, choices=data.choices),
                                transitions={   'heard'            :   'SAY_NEXT_QUESTION_4',
                                                'not_heard'        :   'SAY_NEXT_QUESTION_4'}) 

        smach.StateMachine.add( 'SAY_NEXT_QUESTION_4',
                                states.Say(robot, ["What is your next question?"], block=True),
                                #transitions={   'spoken'            :   'GO_TO_EXIT'})
                                transitions={   'spoken'            :   'HEAR_QUESTION_5'})

        smach.StateMachine.add( 'HEAR_QUESTION_5',
                                HearQuestion(robot, spec=data.spec, choices=data.choices),
                                transitions={   'heard'            :   'SAY_GOTO_EXIT',
                                                'not_heard'        :   'SAY_GOTO_EXIT'}) 

        smach.StateMachine.add( 'SAY_GOTO_EXIT',
                                states.Say(robot, ["I will move to the exit now. See you guys later!"], block=True),
                                #transitions={   'spoken'            :   'GO_TO_EXIT'})
                                transitions={   'spoken'            :   'Done'})

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT',
                                states.NavigateToWaypoint(robot, Designator("exit_1_rips"), radius = 0.5),
                                transitions={   'arrived'           :   'AT_END',
                                                'unreachable'       :   'AT_END',
                                                'goal_not_defined'  :   'AT_END'})

        # Finally amigo will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                                states.Say(robot, "Goodbye"),
                                transitions={   'spoken'            :   'Done'})
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('challenge_speech_recognition_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE SPEECH RECOGNITION] Please provide robot name as argument."
        exit(1)

    states.util.startup(setup_statemachine, robot_name=robot_name)
