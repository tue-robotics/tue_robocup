from typing import Union

import smach
import rospy

from robot_skills.robot import Robot


class NavigateWiggle(smach.State):
    def __init__(
        self,
        robot: Robot,
        duration: Union[rospy.Duration, float, int],
        vth: Union[float, int] = 0.5,
        angle: Union[float, int] = 0.15,
        speak: bool = True,
    ):
        """
        Wiggle the base of the robot. It rotates clockwise to :param: angle, followed by rotating anti-clockwise to
        minus :param: angle. This is repeated until it has to return to its original orientation.
        The decision is taken at each change of direction.

        :param robot: Robot instance
        :param duration: Duration of the wiggle
        :param vth: Angular speed
        :param angle: Angle of the wiggle from the center
        :param speak: Flag whether the robot should speak
        """
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        # Take into account the robot has to return to original orientation
        return_duration = rospy.Duration.from_sec(angle / vth)
        self.duration = rospy.Duration(duration) - return_duration
        self.vth = vth
        self.angle = angle
        self.speak = speak

    def execute(self, userdata=None):
        pose = 0  # pose: 0, 1, -1, 1, -1, ..., 0
        turn = -1  # Start rotating clockwise

        start = rospy.Time.now()
        while not rospy.is_shutdown() and ((time_left := (rospy.Time.now() - start) < self.duration) or abs(pose)):
            if self.speak:
                self.robot.speech.speak("Wiggle wiggle", block=False)

            multiplier = 1
            if time_left:
                multiplier += abs(pose)

            self.robot.base.force_drive(0, 0, turn * self.vth, (multiplier * self.angle) / self.vth)
            pose += multiplier * turn
            turn = -turn

        return "done"


if __name__ == "__main__":
    from robot_skills import get_robot
    import os.path

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero", 1)
    sm = NavigateWiggle(robot_instance, 3.5, 1, 1.5)
    sm.execute()
