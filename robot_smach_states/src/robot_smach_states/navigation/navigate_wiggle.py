import smach
import rospy

class NavigateWiggle(smach.State):
    def __init__(self, robot, duration):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.duration = duration

    def execute(self, userdata=None):
        start = None
        last_say = None

        turn = -1

        current_seconds = rospy.Time.now().to_sec()

        radians = 0.15
        vth = 0.5
        if start is None:
            self.robot.base.force_drive(0, 0, vth, radians / vth)
            start = current_seconds

        dt = current_seconds - start

        if dt > self.duration:
            return "done"

        if last_say is None or current_seconds - last_say > 10:
            self.robot.speech.speak("Trying for another %d seconds .. wiggle wiggle" % int(self.duration - dt), block=False)
            last_say = current_seconds

        self.robot.base.force_drive(0, 0, turn * vth, (2 * radians) / vth)
        turn = -turn

        return "done"
