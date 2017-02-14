#! /usr/bin/env python
import rospy
import smach


class ResetPart(smach.State):
    def __init__(self, robot, part, timeout=0.0):
        self.robot = robot
        self.part = part
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        if self.timeout:
            self.part.reset(self.timeout)
        else:
            self.part.reset()
        return 'done'


class ResetHead(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        rospy.logwarn("Better use another function, what do you want to achieve, straight or cancel???")
        self.robot.head.reset(timeout=self.timeout)
        return "done"


class CancelHead(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        rospy.logwarn("Better use another function, what do you want to achieve, straight or cancel???")
        self.robot.head.cancel_goal()
        return "done"


class ResetArms(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        self.robot.leftArm.reset(timeout=self.timeout)
        self.robot.leftArm.send_gripper_goal('close', timeout=self.timeout)
        self.robot.rightArm.reset(timeout=self.timeout)
        self.robot.rightArm.send_gripper_goal('close', timeout=self.timeout)
        return "done"


class ResetED(smach.State):
    def __init__(self, robot):
        self.robot = robot
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        self.robot.lights.set_color(r=1.0,g=0.0,b=0.0,a=1.0) # Red
        self.robot.ed.reset()                                # Reset ed
        self.robot.lights.set_color(r=0.0,g=0.0,b=1.0,a=1.0) # Blue
        return 'done'


class ResetArmsSpindle(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):

        self.robot.leftArm.reset()
        self.robot.leftArm.send_gripper_goal('close', timeout=self.timeout)
        self.robot.rightArm.reset()
        self.robot.rightArm.send_gripper_goal('close', timeout=self.timeout)
        self.robot.spindle.reset()
        return "done"


class ResetArmsSpindleHead(smach.State):
    # Checks how many tasks have been done and if another task is needed
    # Does this check with the database in the reasoner
    def __init__(self,robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):

        self.robot.leftArm.reset()
        self.robot.leftArm.send_gripper_goal('close', timeout=self.timeout)
        self.robot.rightArm.reset()
        self.robot.rightArm.send_gripper_goal('close', timeout=self.timeout)
        self.robot.head.reset(timeout=self.timeout)
        self.robot.spindle.reset()

        return "done"
