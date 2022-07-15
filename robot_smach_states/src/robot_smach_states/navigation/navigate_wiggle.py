import smach
import rospy

class NavigateWiggle(smach.State):
    def __init__(self, robot, duration, speak=True):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.duration = duration
        self.speak = speak

    def execute(self, userdata=None):
        start = None

        turn = -1

        start = rospy.Time.now()
        vth = 0.5
        radians = 0.15
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < self.duration:
            if self.speak:
                self.robot.speech.speak("Wiggle wiggle", block=False)
            self.robot.base.force_drive(0, 0, turn * vth, (2 * radians) / vth)
            turn = -turn

        return "done"


if __name__ == "__main__":
    from robot_skills import get_robot
    import os.path
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot = get_robot("hero")
    sm = NavigateWiggle(robot, 1)
    sm.execute()
