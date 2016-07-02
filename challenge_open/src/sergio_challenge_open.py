#!/usr/bin/env python

import rospy, sys, robot_smach_states, random
from robot_skills.sergio import Sergio as Robot

if __name__ == "__main__":
    rospy.init_node('challenge_open_sergio')

    robot = Robot()

    robot.speech.speak("No ids specified, I will do them all", block=False)
    ids = [e.id for e in robot.ed.get_entities() if e.has_shape and "sergio" not in e.id and "walls" not in e.id and "floor" not in e.id]
    random.shuffle(ids)

    print "IDS:", ids

    for id in ids:

        robot.speech.speak("I am going to navigate to the %s" % id, block=False)

        machine = robot_smach_states.NavigateToSymbolic(robot, {robot_smach_states.util.designators.EntityByIdDesignator(robot, id=id): "near"},
                                                        robot_smach_states.util.designators.EntityByIdDesignator(robot, id=id))

        result = machine.execute()

        if result == "arrived":
            robot.speech.speak("I arrived at the %s" % id, block=True)
        else:
            robot.speech.speak("I failed to navigate to the %s" % id, block=True)
