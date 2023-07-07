import os
from datetime import datetime

import cv2
from cv_bridge import CvBridge

from ed.entity import Entity
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.checks import is_writeable
from robot_smach_states.util.designators.utility import value_or_resolve

from pykdl_ros import VectorStamped

import smach


class GetCustomerImage(smach.State):

    def __init__(self, robot, customer_designator: Designator[Entity], image_designator: Designator[str]):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._customer_designator = customer_designator
        is_writeable(image_designator)
        self._image_designator = image_designator
        self._bridge = CvBridge()

    def execute(self, userdata=None):
        customer = value_or_resolve(self._customer_designator)  # type: Entity
        if not customer:
            return "failed"
        head_pose = VectorStamped.from_framestamped(customer.pose)
        head_pose.vector.z(head_pose.vector.z() + 0.1)
        self._robot.head.look_at_point(head_pose)
        self._robot.head.wait_for_motion_done()
        rgb, _, _ = self._robot.perception.get_rgb_depth_caminfo()
        rgb_cv = self._bridge.imgmsg_to_cv2(rgb, "bgr8")
        rgb_cv = rgb_cv[240-100:240+100, 320-100:320+100, :]  # TooD: make dynamic

        os.makedirs(os.path.expanduser(os.path.join("/tmp", "restaurant")), exist_ok=True)
        filename = os.path.expanduser(
            os.path.join("/tmp", "restaurant", f"face-{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.png")
        )
        self._robot.head.cancel_goal()

        if not cv2.imwrite(filename, rgb_cv):
            return "failed"

        self._image_designator.write(filename)

        return "succeeded"
