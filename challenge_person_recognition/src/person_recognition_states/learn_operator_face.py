import smach
import robot_smach_states as states
import robot_smach_states.util.designators as ds


class LearnOperatorFace(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot

        @smach.cb_interface(outcomes=['done'])
        def look_at_standing_person(userdata=None):
            robot.head.look_at_standing_person()
            return 'done'

        with self:
            smach.StateMachine.add( 'LOOK_AT_OPERATOR',
                                    smach.CBState(look_at_standing_person),
                                    transitions={'done': 'SAY_LOOK_AT_ME'})

            smach.StateMachine.add( 'SAY_LOOK_AT_ME',
                                    states.Say(robot, "Please stand one meter in front of me and look at me while I \
                                    learn to recognize your face.", block=True), transitions={'spoken': 'LEARN_PERSON'})

            smach.StateMachine.add('LEARN_PERSON',
                                   states.LearnPerson(robot, name_designator=ds.VariableDesignator("operator")),
                                   transitions={'succeeded_learning': 'SAY_OPERATOR_LEARNED',
                                                'failed_learning': 'SAY_FAILED_LEARNING',
                                                'timeout_learning': 'SAY_FAILED_LEARNING'})

            smach.StateMachine.add('SAY_FAILED_LEARNING',
                                   states.Say(robot, "I could not learn my operator's face. Let me try again.",
                                              block=True), transitions={'spoken': 'failed'})

            smach.StateMachine.add('SAY_OPERATOR_LEARNED',
                                   states.Say(robot, "Now i know what you look like. Please go mix with the crowd."),
                                   transitions={'spoken': 'succeeded'})
