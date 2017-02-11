#! /usr/bin/env python
import rospy
import smach

############################## Atomic Reset States ##############################

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

class ResetLeftArm(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        self.robot.leftArm.reset()
        self.robot.leftArm.send_gripper_goal('close', timeout=self.timeout)
        return "done"

class ResetRightArm(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        self.robot.rightArm.reset()
        self.robot.rightArm.send_gripper_goal('close', timeout=self.timeout)
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

class ResetArm(smach.State):
    def __init__(self, robot, side, timeout=0.0):
        """
        :param side an arm of the robot or a designator resolving to an Arm (some ArmDesignator)
        """
        self.robot = robot
        self.timeout = timeout
        self.side = side
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        if hasattr(self.side, "resolve"):
            side = self.side.resolve()
        else:
            side = self.side
        side.reset()
        side.send_gripper_goal('close', timeout=self.timeout)
        return "done"

class ResetSpindle(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        self.robot.spindle.reset()
        return "done"

class ResetTorso(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        self.robot.torso.reset()
        return "done"

class SetSpindle(smach.State):
    def __init__(self, robot, timeout=0.0, height=0.35):
        """
        :param height Height to which to set the spindle
        """
        self.robot = robot
        self.timeout = timeout
        self.height = height
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):
        self.robot.spindle.send_goal(self.height)
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
############################## Combination Reset smach.States ##############################

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

class ResetArmsHead(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):

        self.robot.leftArm.reset()
        self.robot.leftArm.send_gripper_goal('close', timeout=self.timeout)
        self.robot.rightArm.reset()
        self.robot.rightArm.send_gripper_goal('close', timeout=self.timeout)
        self.robot.head.reset(timeout=self.timeout)
        return "done"

class ResetHeadSpindle(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):

        self.robot.spindle.reset()
        self.robot.head.reset(timeout=self.timeout)
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

class ResetSpindleHeadUp(smach.State):
    def __init__(self, robot, timeout=0.0):
        self.robot = robot
        self.timeout = timeout
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata=None):

        self.robot.spindle.reset()
        self.robot.head.reset()
        return "done"