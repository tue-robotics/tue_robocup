#! /usr/bin/env python
import rospy

from robot_smach_states.util.startup import startup
import smach

import robot_smach_states as states

from psi import Compound, Conjunction, Sequence

class Boo(smach.StateMachine):
    """Amigo will look into the distance doing nothing. 
        From experience, we know people will start looking into his 'eyes'.
        If all is well, a face will be recognized and Amigo will say 'boo!' 
            to scare the kids that is now close to the robot"""
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Failed", "Aborted"])
        self.robot = robot

        self.query_detect_face = Conjunction(Compound("property_expected", "ObjectID", "class_label", "face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        with self:
            #TODO: Make sure the arms are straight down and the head is staring away into the distance
            smach.StateMachine.add( "RESET_RIGHT",
                                    states.ArmToJointPos(robot, robot.rightArm, [0,0,0,0,0,0,0]),
                                    transitions={"done":"RESET_LEFT", "failed":"RESET_LEFT"})
            smach.StateMachine.add( "RESET_LEFT",
                                    states.ArmToJointPos(robot, robot.leftArm, [0,0,0,0,0,0,0]),
                                    transitions={"done":"WAIT_FOR_PERSON", "failed":"WAIT_FOR_PERSON"})

            smach.StateMachine.add("WAIT_FOR_PERSON",
                    states.Wait_queried_perception(self.robot, ["face_recognition"], self.query_detect_face, timeout=60),
                    transitions={   "query_true":"TALK",
                                    "timed_out":"Failed",
                                    "preempted":"Aborted"})

            smach.StateMachine.add("TALK",
                    states.Say(robot, ["Boooo!"]),
                    transitions={   'spoken'      :'Done'})

if __name__ == "__main__":
    rospy.init_node("challenge_robo_zoo_boo")

    startup(Boo)
