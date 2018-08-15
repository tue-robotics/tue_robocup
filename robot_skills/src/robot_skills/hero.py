import robot

import rospy
import geometry_msgs.msg

from collections import OrderedDict


class Hero(robot.Robot):
    """docstring for Hero"""
    def __init__(self, wait_services=False):
        super(Hero, self).__init__(robot_name="hero", wait_services=wait_services)

        self._ignored_parts = ["leftArm", "rightArm", "torso", "spindle", "head"]

        # remap topics in base
        self.parts['base']._cmd_vel = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)

    	#rename joint names
	    self.parts['leftArm'].joint_names = self.parts['leftArm'].load_param('skills/arm/joint_names')
        self.parts['rightArm'].joint_names = self.parts['rightArm'].load_param('skills/arm/joint_names')

        # This is still very ugly, because there is a lof of double code, but atleast it is only in sergio.
        #self.parts['rightArm'] = robot.arms.FakeArm(self.robot_name, self.tf_listener, side="right")

        #for partname, bodypart in self.parts.iteritems():
        #    setattr(self, partname, bodypart)

        self.arms = OrderedDict(left=self.leftArm, right=self.rightArm)
