#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')
import rospy 
import smach

import robot_smach_states as states
from look_at_person import LookAtPerson

from psi import Compound, Conjunction, Sequence

lines = ["Roses are red, violets are blue, all my base, are belong to you.",
         "Was your father a thief? Because he stole some titanium bolts and put them in your eyes.",
         "Do you believe in love at first optical recognition, or should I scan your location again?",
         "Were you designed for use on Mars? Because your chassis is out of this world!",
         "Was that my CPU malfunctioning or did I just feel a spark between us?",
         "Can I have your ip number? I seem to have lost mine."]

class Pickup(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done"])
        self.robot = robot
        self.query_detect_face = Conjunction(Compound("property_expected", "ObjectID", "class_label", "face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))
                                          
        with self:
            smach.StateMachine.add("LOOK_AT_PERSON",
                    LookAtPerson(robot),
                    transitions={   'Done'      :'SAY_HI',
                                    'Aborted'   :'SAY_HI',
                                    'Failed'    :'SAY_HI'})

            # smach.StateMachine.add("WAIT_FOR_PERSON",
            #         states.Wait_queried_perception(self.robot, ["face_recognition"], self.query_detect_face, timeout=10),
            #         transitions={   "query_true":"SAY_HI",
            #                          "timed_out":"SAY_HI",
            #                          "preempted":"SAY_HI"})
                                    
            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["Hi there, how are you doing?"]),
                                    transitions={"spoken":"SAY_PICKUPLINE"})

            smach.StateMachine.add( "SAY_PICKUPLINE",
                                    states.Say(robot, lines),
                                    transitions={"spoken":"Done"})
