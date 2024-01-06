# System
import functools
import math
import typing
from threading import Condition, Event
import os
from datetime import datetime
from cv_bridge import CvBridge

import cv2

# ROS
import message_filters
import PyKDL as kdl
import rospy
from image_recognition_msgs.msg import Annotation

# TU/e Robotics
from image_recognition_msgs.msg import Recognition
from image_recognition_msgs.srv import Annotate, GetFaceProperties, Recognize, RecognizeResponse
from people_recognition_msgs.srv import RecognizePeople3D
from rgbd_msgs.srv import Project2DTo3D
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Empty
from tue_msgs.msg import LocateDoorHandleAction

from pykdl_ros import VectorStamped
# Robot skills
from robot_skills.robot_part import RobotPart
from robot_skills.util.image_operations import img_cutout, img_recognitions_to_rois


class Perception(RobotPart):
    def __init__(
        self,
        robot_name,
        tf_buffer,
        image_topic=None,
        projection_srv=None,
        camera_base_ns='',
        robot_base_frame_id=None,
    ):
        super(Perception, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        if image_topic is None:
            self.image_topic = "/" + self.robot_name + "/top_kinect/rgb/image"
        else:
            self.image_topic = image_topic

        if projection_srv is None:
            projection_srv_name = '/' + robot_name + '/top_kinect/project_2d_to_3d'
        else:
            projection_srv_name = projection_srv

        self._camera_base_ns = camera_base_ns

        self._camera_lazy_sub = None
        self._camera_cv = Condition()
        self._camera_last_image = None

        self._annotate_srv = self.create_service_client(
            '/' + robot_name + '/people_recognition/face_recognition/annotate', Annotate)
        self._recognize_srv = self.create_service_client(
            '/' + robot_name + '/people_recognition/face_recognition/recognize', Recognize)
        self._clear_srv = self.create_service_client(
            '/' + robot_name + '/people_recognition/face_recognition/clear', Empty)

        self._image_data = (None, None, None)

        self._face_properties_srv = self.create_service_client(
            '/' + robot_name + '/people_recognition/face_recognition/get_face_properties', GetFaceProperties)

        self._projection_srv = self.create_service_client(projection_srv_name, Project2DTo3D)
        self._person_recognition_3d_srv = \
            self.create_service_client('/' + robot_name + '/people_recognition/detect_people_3d', RecognizePeople3D)

        # self._locate_handle_client = self.create_simple_action_client('/' + robot_name + '/handle_locator/locate_handle', LocateDoorHandleAction)

        self._robot_base_frame_id = robot_base_frame_id if robot_base_frame_id else f"{robot_name}/base_link"

    def _image_cb(self, image):
        self._camera_cv.acquire()
        self._camera_last_image = image
        self._camera_cv.notify()
        self._camera_cv.release()

    def get_image(self, timeout=5):
        # lazy subscribe to the rgb(d) camera
        if not self._camera_lazy_sub:
            rospy.loginfo("Creating subscriber")
            self._camera_lazy_sub = rospy.Subscriber(self.image_topic, Image, self._image_cb)
            rospy.loginfo('lazy subscribe to %s', self._camera_lazy_sub.name)

        rospy.loginfo("getting one image...")
        self._camera_cv.acquire()
        self._camera_last_image = None
        for i in range(timeout):
            if self._camera_last_image:
                rospy.loginfo("len(self._camera_last_image): {}".format(len(self._camera_last_image.data)))
                break
            else:
                rospy.loginfo("self._camera_last_image: {}".format(self._camera_last_image))

            if rospy.is_shutdown():
                return

            self._camera_cv.wait(timeout=1)
        else:
            raise Exception('no image received from %s' % self._camera_lazy_sub.name)

        image = self._camera_last_image
        self._camera_cv.release()

        rospy.loginfo("got %d bytes of image data", len(image.data))
        return image

    def project_roi(self, roi, frame_id=None):
        """ Projects a region of interest of a depth image to a 3D Point. Hereto, a service is used

        :param roi: sensor_msgs/RegionOfInterest
        :param frame_id: if specified, the result is transformed into this frame id
        :return: VectorStamped object
        """
        response = self.project_rois(rois=[roi]).points[0]

        # Convert to VectorStamped
        result = VectorStamped.from_xyz(response.point.x, response.point.y, response.point.z, rospy.Time.now(),
                                        response.header.frame_id)

        # If necessary, transform the point
        if frame_id is not None:
            rospy.loginfo("Transforming roi to {}".format(frame_id))
            result = self.tf_buffer.transform(result, frame_id, timeout=rospy.Duration(0.5))

        return result

    def project_rois(self, rois):
        # Call the service with the provided Region of Interest
        try:
            points = self._projection_srv(rois=rois)
        except rospy.ServiceException as e:
            raise ValueError('project_roi failed', e)
        else:
            rospy.loginfo('project_rois response: %s', points)
            return points

    # OpenFace
    def _get_faces(self, image=None):
        if not image:
            image = self.get_image()
        try:
            r = self._recognize_srv(image=image)
            rospy.loginfo('Found %d face(s) in the image', len(r.recognitions))
        except rospy.ServiceException as e:
            rospy.logerr("Can't connect to face recognition service: {}".format(e))
            r = RecognizeResponse()
        except Exception as e:
            rospy.logerr("Can't detect faces: {}".format(e))
        return r

    def learn_person(self, name='operator', robot=None):

        HEIGHT_TRESHOLD = 88
        WIDTH_TRESHOLD = 88
        try:
            image = self.get_image()
        except Exception as e:
            rospy.logerr("Can't get image: {}".format(e))
            return False

        raw_recognitions = self._get_faces(image).recognitions
        recognitions = [r for r in raw_recognitions if r.roi.height > HEIGHT_TRESHOLD and r.roi.width > WIDTH_TRESHOLD]
        rospy.loginfo('Found %d valid face(s)', len(recognitions))

        if len(recognitions) != 1:
            rospy.loginfo("Too many faces: {}".format(len(recognitions)))
            return False

        recognition = recognitions[0]

        rospy.loginfo('annotating that face as %s', name)
        try:
            self._annotate_srv(image=image, annotations=[Annotation(label=name, roi=recognition.roi)])
        except rospy.ServiceException as e:
            rospy.logerr("Can't connect to person learning service: {}".format(e))
            return False
        except Exception as e:
            rospy.logerr("Can't learn a person: {}".format(e))
            return False

        if robot:
            bridge = CvBridge()
            rgb_cv = bridge.imgmsg_to_cv2(image, "bgr8")

            os.makedirs(os.path.expanduser(os.path.join("/tmp", "learn_person")), exist_ok=True)
            filename = os.path.expanduser(
                os.path.join("/tmp", "learn_person", f"face-{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.png")
            )

            if not cv2.imwrite(filename, rgb_cv):
                return False

            robot.hmi.show_image(filename, 10)

        return True

    def detect_faces(self, image=None):
        """
        Snap an image with the camera and return the recognized faces.

        :param image: image to use for recognition
        :type image: sensor_msgs/Image
        :return: recognitions of the faces
        :rtype: tuple(list[image_recognition_msgs/Recognition], stamp)
        """
        if not image:
            image = self.get_image()

        return self._get_faces(image).recognitions, image.header.stamp

    def detect_operator_face(
        self,
        image: typing.Optional[Image] = None,
        expected_operator_position: typing.Optional[kdl.Vector] = None,
        operator_distance_threshold: typing.Optional[float] = 0.5,
    ) -> Recognition:
        """
        Snap an image with the camera and return the detected face closest to the expected position.

        N.B.: this seems more reliable than using the size of the ROI, which might be affected by, e.g., a big hairdo.

        :param image: image to use for recognition
        :param expected_operator_position: expected operator position w.r.t. robot base
        :param operator_distance_threshold: people outside this radius from the expected position are discarded
        :return: image_recognition_msgs/Recognition
        :raises: RuntimeError
        """
        recognitions, _ = self.detect_faces(image)
        if not recognitions:
            raise RuntimeError("No faces detected")
        operator_recognition = self._filter_operator_recognition(
            recognitions, expected_operator_position, operator_distance_threshold
        )
        return operator_recognition

    def _filter_operator_recognition(
        self,
        recognitions: typing.List[Recognition],
        expected_operator_pos: typing.Optional[kdl.Vector] = None,
        threshold: float = 0.5,
    ) -> Recognition:
        """
        For all provided recognitions, project the ROI and determine the one closest to the expected operator position,
        computed on the floor plane.

        :param recognitions: recognitions
        :param expected_operator_pos: expected position of the operator w.r.t. the robot
        :param threshold: if the distance between the recognition and the expected pos exceed the threshold, a Runtime error
        is raised
        :return: Recognition closed to the expected position
        :raises: RuntimeError
        """
        projected_recognitions = [(rcg, self.project_roi(rcg.roi, self._robot_base_frame_id)) for rcg in recognitions]
        expected_operator_pos = kdl.Vector(1.0, 0.0, 0.0) if expected_operator_pos is None else expected_operator_pos

        def _distance_from_expected(
            expected_pos: kdl.Vector,
            projected_tup: typing.Tuple[Recognition, VectorStamped]
        ) -> float:
            """
            Helper method to compute the distance between the projected tuple and the expected position

            :param expected_pos: expected operator position w.r.t. the robot
            :param projected_tup: tuple contains the original recognition as well as a VectorStamped of the projected
            ROI
            :return: distance between the expected position and the projected ROI
            """
            dx = projected_tup[1].vector.x() - expected_pos.x()
            dy = projected_tup[1].vector.y() - expected_pos.y()
            return math.hypot(dx, dy)

        projected_recognitions.sort(key=functools.partial(_distance_from_expected, expected_operator_pos))
        best_distance = _distance_from_expected(expected_operator_pos, projected_recognitions[0])
        if best_distance > threshold:
            err_msg = f"Distance between face detection and expected pos {best_distance} exceeds threshold {threshold}"
            rospy.logwarn(err_msg)
            raise RuntimeError(err_msg)
        else:
            return projected_recognitions[0][0]

    @staticmethod
    def get_best_face_recognition(recognitions, desired_label, probability_threshold=4.0):
        """
        Returns the Recognition with the highest probability of having the desired_label.
        Assumes that the probability distributions in Recognition are already sorted by probability (descending, highest first)

        :param recognitions: The recognitions to select the best one with desired_label from
        :type recognitions: list[image_recognition_msgs/Recognition]
        :param desired_label: what label to look for in the recognitions
        :type desired_label: str
        :param probability_threshold: only accept recognitions with probability higher than threshold
        :type probability_threshold: double
        :return: the best recognition matching the given desired_label
        :rtype image_recognition_msgs/Recognition
        """

        rospy.logdebug("get_best_face_recognition: recognitions = {}".format(recognitions))

        # Only take detections with operator
        # detections = []
        # The old implementation took, for each recognition, the (label, prob) pairs where label==desired_label.
        # Other pairs in the same distribution may have higher probability.
        # When the best_recognition is picked, it picked the recognition where the probability for the desired_label is hhighest comapared to other recognitions. BUT: a recognitions highest probability may be for a different label
        # because the selection only compares matching labels, not looking at the probability of non-matching pairs.
        # For example: we have 2 recognitions.
        #   in recognition 1, A has 50%, desired_label has 30%, B has 20%.
        #   in recognition 2, B has 60%, desired_label has 35%, A has 5%.
        # Then, recognition 2 has the highest probability for the desired_label and is thus picked.
        # Because we take the [0]'th index of the distribution, that name is B
        #
        # Solution: because the probability distributions are sorted, just take the probability distribution where the desired label has the highest probability.
        # for recog in recognitions:
        #    for cp in recog.categorical_distribution.probabilities:
        #        if cp.label == desired_label:
        #            detections.append((recog, cp.probability))

        # Sort based on probability
        # if detections:
        #    sorted_detections = sorted(detections, key=lambda det: det[1])
        #    best_detection = sorted_detections[0][0]  # A CategoricalDistribution in a Recognition is already ordered, max prob is at [0]
        # else:
        #    best_detection = None

        rospy.loginfo("Probability threshold %.2f", probability_threshold)
        for index, recog in enumerate(recognitions):
            rospy.loginfo("{index}: {dist}".format(index=index,
                                                   dist=[(cp.label, "{:.2f}".format(cp.probability)) for cp in
                                                         recog.categorical_distribution.probabilities]))

        matching_recognitions = [recog for recog in recognitions if \
                                 recog.categorical_distribution.probabilities and \
                                 recog.categorical_distribution.probabilities[0].label == desired_label]

        if matching_recognitions:
            best_recognition = max(matching_recognitions,
                                   key=lambda recog: recog.categorical_distribution.probabilities[0].probability)
            return best_recognition if best_recognition.categorical_distribution.probabilities[
                                           0].probability > probability_threshold else None
        else:
            return None  # TODO: Maybe so something smart with selecting a recognition where the desired_label is not the most probable for a recognition?

    def clear_face(self):
        """
        clearing all faces from the OpenFace node.

        :return: no return
        """
        rospy.loginfo('clearing all learned faces')
        self._clear_srv()

    # Skybiometry
    def get_face_properties(self, faces=None, image=None):
        """
        Get the face properties of all faces or in an image. If faces is provided, image is ignored. If both aren't
        provided, an image is collected.

        :param faces: images of all faces
        :type faces: list[sensor_msgs/Image]
        :param image: image containing the faces
        :type image: sensor_msgs/Image
        :return: list of face properties
        :rtype: list[image_recognition_msgs/FaceProperties]
        """
        if not faces:
            if not image:
                image = self.get_image()
            face_recognitions, _ = self.detect_faces(image=image)
            rois = img_recognitions_to_rois(face_recognitions)
            faces = img_cutout(image, rois)

        try:
            face_properties_response = self._face_properties_srv(faces)
            face_properties = face_properties_response.properties_array
        except Exception as e:
            rospy.logerr(e)
            return [None] * len(faces)

        face_log = '\n - '.join([''] + [repr(s) for s in face_properties])
        rospy.loginfo('face_properties:%s', face_log)
        return face_properties

    def get_rgb_depth_caminfo(self, timeout=5):
        """
        Get an rgb image and depth image, along with camera info for the depth camera.
        The returned tuple can serve as input for world_model_ed.ED.detect_people.

        :param timeout: How long to wait until the images are all collected.
        :return: tuple(rgb, depth, depth_info) or a None if no images could be gathered.
        """
        event = Event()

        def callback(rgb, depth, depth_info):
            rospy.loginfo('Received rgb, depth, cam_info')
            self._image_data = (rgb, depth, depth_info)
            event.set()

        # camera topics
        depth_info_sub = message_filters.Subscriber('{}/depth_registered/camera_info'.format(self._camera_base_ns),
                                                    CameraInfo)
        depth_sub = message_filters.Subscriber('{}/depth_registered/image'.format(self._camera_base_ns), Image)
        rgb_sub = message_filters.Subscriber('{}/rgb/image_raw'.format(self._camera_base_ns), Image)

        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, depth_info_sub],
                                                         queue_size=1,
                                                         slop=10)
        ts.registerCallback(callback)
        event.wait(timeout)
        ts.callbacks.clear()
        del ts, depth_info_sub, depth_sub, rgb_sub, callback

        if any(self._image_data):
            return self._image_data
        else:
            return None, None, None

    def detect_person_3d(self, rgb, depth, depth_info):
        return self._person_recognition_3d_srv(image_rgb=rgb, image_depth=depth, camera_info_depth=depth_info).people

    # @property
    # def locate_handle_client(self):
    #     return self._locate_handle_client
