#!/usr/bin/env python
import rospy
import smach
import smach_ros
import robot_smach_states as states
from robocup_knowledge import load_knowledge
from robot_skills import Hero
challenge_knowledge = load_knowledge('challenge_help_me_carry')
if __name__ == '__main__':
    rospy.init_node('Hello_Hero')
    robot = Hero()
    # Create a smach state machine
    sm = smach.StateMachine(outcomes=['Completed', 'Aborted'])
    # Open the container
    with sm:
        smach.StateMachine.add('INITIALIZE',
                               states.utility.Initialize(robot),
                               transitions={'initialized': 'SET_INITIAL_POSE',
                                            'abort': 'Aborted'})
        smach.StateMachine.add('SET_INITIAL_POSE',
                               states.utility.SetInitialPose(robot,challenge_knowledge.starting_point),
                               transitions={'done': 'ASK_FOR_BAG',
                                            "preempted": 'Aborted',
                                            'error': 'Aborted'})
        smach.StateMachine.add('ASK_FOR_BAG',
                               states.human_interaction.Say(robot, ["Which bags do I carry?"],
                                          block=True,
                                          look_at_standing_person=True),
                               transitions={'spoken': 'Aborted'})
    sis = smach_ros.introspection.IntrospectionServer('Simplestatemachine', sm, '/SimpleSM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()
