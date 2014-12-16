#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import amigo

if __name__ == "__main__":
    import atexit
    import util.msg_constructors as msgs
    from reasoner import Compound, Conjunction

    rospy.init_node("dummy_sayer")
    amigo = amigo.Amigo(wait_services=False)
    robot = amigo #All state machines use robot. ..., this makes copy/pasting easier.

    for i in range(99,0, -1):
        try:
            robot.speech.speak("{0} bottles of beer on the wall, {0} bottles of beer.".format(i))
            rospy.sleep(0.5)
        except KeyboardInterrupt:
            break