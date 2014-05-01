#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')

import smach

import robot_smach_states as states

class LookAtPerson(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        with self:
            smach.StateMachine.add("RESET_SPINDLE_HEAD_UP",
                    states.ResetSpindle_HeadUp(robot),
                    transitions={'done':'LOOK'})

            smach.StateMachine.add("LOOK",
                    states.LookAtItem(robot, ["face_recognition"], states.LookAtItem.face_in_front_query),
                    transitions={   'Done'      :'Done',
                                    'Aborted'   :'Aborted',
                                    'Failed'    :'Failed'})