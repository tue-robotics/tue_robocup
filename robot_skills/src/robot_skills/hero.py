import robot

import rospy
import geometry_msgs.msg

from collections import OrderedDict
from sensor_msgs.msg import Image

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

        self.arms = OrderedDict(left=self.leftArm, right=self.rightArm)

        # These don't work for HSR because (then) Toyota's diagnostics aggregator makes the robot go into error somehow
        self.leftArm.unsubscribe_hardware_status()
        self.rightArm.unsubscribe_hardware_status()
        self.leftArm._operational = True
        self.rightArm._operational = True

        self.parts['perception']._camera_lazy_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.parts['perception']._image_cb)

        self.laser_topic = "/hsrb/base_scan"

