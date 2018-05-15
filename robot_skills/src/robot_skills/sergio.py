import robot

from collections import OrderedDict


class Sergio(robot.Robot):
    """docstring for Sergio"""
    def __init__(self, wait_services=False):
        super(Sergio, self).__init__(robot_name="sergio", wait_services=wait_services)

        self._ignored_parts = ["leftArm", "rightArm", "torso", "spindle", "head"]

        # This is still very ugly, because there is a lof of double code, but atleast it is only in sergio.
        self.parts['leftArm'] = robot.arms.FakeArm(self.robot_name, self.tf_listener, side="left")
        self.parts['rightArm'] = robot.arms.FakeArm(self.robot_name, self.tf_listener, side="right")

        for partname, bodypart in self.parts.iteritems():
            setattr(self, partname, bodypart)

        self.arms = OrderedDict(left=self.leftArm, right=self.rightArm)
