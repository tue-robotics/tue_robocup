#!/usr/bin/env python

import rospy, sys, robot_smach_states, random
from robot_skills import get_robot_from_argv

if __name__ == "__main__":
    rospy.init_node('navigate_in_front_of')

    robot = get_robot_from_argv(index=1)

    if len(sys.argv) > 2:
        ids = sys.argv[2:]
    else:
        robot.speech.speak("No ids specified, I will do them all", block=False)
        ids = [e.id for e in robot.ed.get_entities() if e.is_a("furniture")]
        random.shuffle(ids)

    print "IDS:", ids

    for id in ids:

        robot.speech.speak("I am going to navigate to the %s" % id, block=False)

        machine = robot_smach_states.NavigateToSymbolic(robot, {robot_smach_states.util.designators.EntityByIdDesignator(robot, id=id): "in_front_of"},
                                                        robot_smach_states.util.designators.EntityByIdDesignator(robot, id=id))

        machine.execute()

        robot.head.look_down()
        robot.head.wait_for_motion_done()

        import time

        time.sleep(1)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Segment
        robot.speech.speak("Segmenting on top of the %s" % id, block=False)

        segmented_entities = robot.ed.update_kinect("on_top_of %s" % id)

        if segmented_entities:
            if not segmented_entities.error_msg:
                robot.speech.speak("I found %d entities" % len(segmented_entities.updated_ids))
            else:
                robot.speech.speak(segmented_entities.error_msg)

        robot.head.close()



