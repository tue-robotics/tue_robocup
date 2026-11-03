"""
Client for a Vision-Language-Action policy server, see https://github.com/paumontagut/per-group-mse-vla

The server holds the model. This part only gathers the observation (head and hand camera image, joint
state and instruction), sends it over a WebSocket and returns the predicted action chunk. Turning the
chunk into commands is up to the caller, see robot_skills.vla_chunk and
robot_smach_states.manipulation.VLAGrasp.
"""
import threading
from typing import Callable, Dict, List, Sequence

import cv2
import msgpack
import numpy as np
import rospy
import websocket
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from robot_skills.robot_part import RobotPart
from robot_skills.vla_chunk import ACTION_DIM


class VLAPolicyError(RuntimeError):
    """Raised when no observation can be made or the policy server does not answer properly"""


class VLAPolicy(RobotPart):
    """Observation gathering and inference requests for a VLA policy server"""

    # Observation layout expected by the HSR-SmolVLA checkpoint
    IMAGE_SHAPE = (480, 640, 3)
    STATE_DIM = 8

    def __init__(self, robot_name: str, tf_buffer, get_joint_states: Callable[[], Dict[str, float]],
                 arm_joint_names: Sequence[str], head_image_topic: str, hand_image_topic: str) -> None:
        """
        Constructor

        :param robot_name: robot name
        :param tf_buffer: tf2_ros.Buffer
        :param get_joint_states: function returning the current joint states as a dict
        :param arm_joint_names: names of the five arm joints, in the order of the policy's arm columns
        :param head_image_topic: topic of the head camera image
        :param hand_image_topic: topic of the hand camera image
        """
        super(VLAPolicy, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._get_joint_states = get_joint_states
        self.arm_joint_names = list(arm_joint_names)
        self.gripper_joint_name = self.load_param("skills/vla/gripper_joint", "hand_motor_joint")
        self.head_joint_names = self.load_param("skills/vla/head_joints", ["head_pan_joint", "head_tilt_joint"])
        self.server_url = self.load_param("skills/vla/server_url", "ws://localhost:8000")
        self.request_timeout = self.load_param("skills/vla/request_timeout", 10.0)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._images = {}  # camera name -> (Image, receive time)
        # Plain subscribers on purpose: the cameras may not be running and this optional part must not delay
        # the start-up of the robot. Freshness is checked when an observation is requested.
        self._subscribers = [
            rospy.Subscriber(head_image_topic, Image, self._image_callback, callback_args="head",
                             queue_size=1, buff_size=2 ** 24),
            rospy.Subscriber(hand_image_topic, Image, self._image_callback, callback_args="hand",
                             queue_size=1, buff_size=2 ** 24),
        ]
        self._ws = None

    @property
    def state_joint_names(self) -> List[str]:
        """Joint names in the order of the policy's state vector"""
        return self.arm_joint_names + [self.gripper_joint_name] + list(self.head_joint_names)

    def close(self) -> None:
        self._disconnect()
        super(VLAPolicy, self).close()

    def _image_callback(self, msg: Image, camera: str) -> None:
        with self._lock:
            self._images[camera] = (msg, rospy.Time.now())

    def get_state(self) -> List[float]:
        """
        Current joint positions in the order of the policy's state vector

        :return: list of STATE_DIM floats
        """
        joint_states = self._get_joint_states()
        try:
            return [joint_states[name] for name in self.state_joint_names]
        except KeyError as e:
            raise VLAPolicyError("Joint {} is not in the joint states".format(e))

    def get_images(self, max_age: float) -> Dict[str, np.ndarray]:
        """
        Latest head and hand camera images as RGB uint8 arrays of IMAGE_SHAPE

        :param max_age: maximum age [s] of the images
        :raises VLAPolicyError: when an image is missing or too old
        """
        with self._lock:
            images = dict(self._images)
        now = rospy.Time.now()

        result = {}
        for camera in ("head", "hand"):
            if camera not in images:
                raise VLAPolicyError("No {} camera image received".format(camera))
            msg, received = images[camera]
            age = (now - received).to_sec()
            if age > max_age:
                raise VLAPolicyError("Last {} camera image is {:.1f} s old".format(camera, age))
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            if image.shape != self.IMAGE_SHAPE:
                image = cv2.resize(image, (self.IMAGE_SHAPE[1], self.IMAGE_SHAPE[0]))
            result[camera] = np.ascontiguousarray(image, dtype=np.uint8)
        return result

    def get_observation(self, instruction: str, max_image_age: float = 1.0) -> dict:
        """
        Assemble the observation for the policy

        :param instruction: natural language task instruction, e.g. "pick up the mug"
        :param max_image_age: maximum age [s] of the camera images
        :return: dict with 'head_rgb', 'hand_rgb', 'state' and 'instruction'
        """
        images = self.get_images(max_image_age)
        return {
            "head_rgb": images["head"],
            "hand_rgb": images["hand"],
            "state": np.asarray(self.get_state(), dtype=np.float32),
            "instruction": instruction,
        }

    def predict(self, instruction: str, max_image_age: float = 1.0) -> np.ndarray:
        """
        Query the policy server for an action chunk given the current observation

        :param instruction: natural language task instruction
        :param max_image_age: maximum age [s] of the camera images
        :return: (T, 11) action chunk, see robot_skills.vla_chunk for the layout
        :raises VLAPolicyError: when no observation can be made or the server fails
        """
        observation = self.get_observation(instruction, max_image_age)
        payload = msgpack.packb({
            "head_rgb": observation["head_rgb"].tobytes(),
            "hand_rgb": observation["hand_rgb"].tobytes(),
            "image_shape": list(self.IMAGE_SHAPE),
            "state": observation["state"].tolist(),
            "instruction": instruction,
        }, use_bin_type=True)

        try:
            ws = self._connect()
            ws.send_binary(payload)
            response = msgpack.unpackb(ws.recv(), raw=False)
        except (websocket.WebSocketException, OSError, ValueError, TypeError) as e:
            self._disconnect()
            raise VLAPolicyError("Request to policy server {} failed: {}".format(self.server_url, e))

        if "error" in response:
            raise VLAPolicyError("Policy server error: {}".format(response["error"]))
        actions = np.asarray(response.get("actions"), dtype=np.float32)
        if actions.ndim != 2 or actions.shape[1] != ACTION_DIM:
            raise VLAPolicyError("Policy server returned actions of shape {}, expected (T, {})"
                                 .format(actions.shape, ACTION_DIM))
        return actions

    def _connect(self):
        if self._ws is None:
            rospy.loginfo("Connecting to policy server at %s", self.server_url)
            self._ws = websocket.create_connection(self.server_url, timeout=self.request_timeout)
        return self._ws

    def _disconnect(self) -> None:
        if self._ws is not None:
            try:
                self._ws.close()
            except (websocket.WebSocketException, OSError):
                pass
            self._ws = None
