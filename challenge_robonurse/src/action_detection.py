import rospy
import smach
import random
import sys


def raw_input_timeout(prompt, timeout=10):
    from select import select

    print prompt
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        s = sys.stdin.readline()
        return s
    else:
        print "No input. Moving on..."

class DetectAction(smach.State):
    def __init__(self, robot, person_to_analyse):
        smach.State.__init__(self, outcomes=["drop_blanket", "fall", "walk_and_sit"])
        self.robot = robot
        self.person_to_analyse = person_to_analyse

    def execute(self, userdata):
        which = raw_input_timeout("Which action has been performed? : {0}".format({i:v for i, v in enumerate(self.get_registered_outcomes())}))
        if which is not None:
            try:
                return self.get_registered_outcomes()[int(which)]
            except:
                rospy.logerr("No valid input received, picking a random action")
                return random.choice(self.get_registered_outcomes())
        else:
            rospy.logerr("No valid input received, picking a random action")
            return random.choice(self.get_registered_outcomes())
