#! /usr/bin/env python
from robot_smach_states.state import State

############################## Atomic Reset States ##############################

class ResetHead(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):
        robot.head.reset(timeout=timeout)
        return "done"

class ResetLeftArm(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):
        robot.leftArm.reset()
        robot.leftArm.send_gripper_goal('close', timeout=timeout)
        return "done"

class ResetRightArm(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):
        robot.rightArm.reset()
        robot.rightArm.send_gripper_goal('close', timeout=timeout)
        return "done"

class ResetArms(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):
        robot.leftArm.reset()
        robot.leftArm.send_gripper_goal('close', timeout=timeout)
        robot.rightArm.reset()
        robot.rightArm.send_gripper_goal('close', timeout=timeout)
        return "done"

class ResetArm(State):
    def __init__(self, robot, side, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, side, timeout):
        side.reset()
        side.send_gripper_goal('close', timeout=timeout)
        return "done"

class ResetSpindle(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):
        robot.spindle.reset()
        return "done"

class ResetTorso(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):
        robot.torso.reset()
        return "done"

class SetSpindle(State):
    def __init__(self, robot, timeout=0.0, height=0.35):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout, height):
        robot.spindle.send_goal(height)
        return "done"
############################## Combination Reset States ##############################

class ResetArmsSpindle(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):

        robot.leftArm.reset()
        robot.leftArm.send_gripper_goal('close', timeout=timeout)
        robot.rightArm.reset()
        robot.rightArm.send_gripper_goal('close', timeout=timeout)
        robot.spindle.reset()
        return "done"

class ResetArmsHead(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):

        robot.leftArm.reset()
        robot.leftArm.send_gripper_goal('close', timeout=timeout)
        robot.rightArm.reset()
        robot.rightArm.send_gripper_goal('close', timeout=timeout)
        robot.head.reset(timeout=timeout)
        return "done"

class ResetHeadSpindle(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):

        robot.spindle.reset()
        robot.head.reset(timeout=timeout)
        return "done"

class ResetArmsSpindleHead(State):
    # Checks how many tasks have been done and if another task is needed
    # Does this check with the database in the reasoner
    def __init__(self,robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):

        robot.leftArm.reset()
        robot.leftArm.send_gripper_goal('close', timeout=timeout)
        robot.rightArm.reset()
        robot.rightArm.send_gripper_goal('close', timeout=timeout)
        robot.head.reset(timeout=timeout)
        robot.spindle.reset()

        return "done"

class ResetSpindleHeadUp(State):
    def __init__(self, robot, timeout=0.0):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, timeout):

        robot.spindle.reset()
        robot.head.reset()
        return "done"