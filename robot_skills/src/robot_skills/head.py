#! /usr/bin/env python
import math
from collections import namedtuple

import rospy
from threading import Condition
from geometry_msgs.msg import PointStamped
from head_ref.msg import HeadReferenceAction, HeadReferenceGoal
from std_srvs.srv import Empty
from image_recognition_msgs.srv import Annotate, Recognize, GetPersons
from image_recognition_msgs.msg import Annotation

from sensor_msgs.msg import Image, RegionOfInterest
from robot_part import RobotPart

# TU/e
from rgbd.srv import Project2DTo3D, Project2DTo3DRequest

from .util import msg_constructors as msgs
from .util.kdl_conversions import kdlVectorStampedToPointStamped, VectorStamped


WavingResult = namedtuple('WavingResult', ['side', 'roi'])


class Skeleton(object):
    """
    nose
    neck
    {left,right}_{shoulder,elbow,wrist}
    """
    def __init__(self, bodyparts):
        pass


class Head(RobotPart):
    def __init__(self, robot_name, tf_listener):
        super(Head, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._ac_head_ref_action = self.create_simple_action_client("/"+robot_name+"/head_ref/action_server",
                                                                    HeadReferenceAction)
        self._annotate_srv = self.create_service_client('/' + robot_name + '/face_recognition/annotate', Annotate)
        self._recognize_srv = self.create_service_client('/' + robot_name + '/face_recognition/recognize', Recognize)
        self._clear_srv = self.create_service_client('/' + robot_name + '/face_recognition/clear', Empty)
        # self._get_persons_srv = self.create_service_client('/detect_persons', GetPersons)
        self._get_persons_srv = self.create_service_client('/' + robot_name + '/person_detection/detect_persons', GetPersons)

        self._projection_srv = self.create_service_client('/' + robot_name + '/top_kinect/project_2d_to_3d',
                                                          Project2DTo3D)

        self._goal = None
        self._at_setpoint = False

    def close(self):
        self._ac_head_ref_action.cancel_all_goals()

    # -- Helpers --

    def reset(self, timeout=0):
        """
        Reset head position
        """
        reset_goal = VectorStamped(x=10, frame_id="/"+self.robot_name+"/base_link")

        return self.look_at_point(reset_goal, timeout=timeout)

    def look_at_hand(self, side):
        """
        Look at the left or right hand, expects string "left" or "right"
        Optionally, keep tracking can be disabled (keep_tracking=False)
        """
        if side == "left":
            return self.look_at_point(VectorStamped(0,0,0,frame_id="/"+self.robot_name+"/grippoint_left"))
        elif side == "right":
            return self.look_at_point(VectorStamped(0,0,0,frame_id="/"+self.robot_name+"/grippoint_right"))
        else:
            rospy.logerr("No side specified for look_at_hand. Give me 'left' or 'right'")
            return False

    def look_at_ground_in_front_of_robot(self, distance=2):
        goal = VectorStamped(x=distance, frame_id="/"+self.robot_name+"/base_link")

        return self.look_at_point(goal)

    def look_down(self, timeout=0):
        """
        Gives a target at z = 1.0 at 1 m in front of the robot
        """
        goal = VectorStamped(1, 0, 0.5, frame_id="/"+self.robot_name+"/base_link")

        return self.look_at_point(goal)

    def look_up(self, timeout=0):
        """
        Gives a target at z = 1.0 at 1 m in front of the robot
        """
        goal = VectorStamped(0.2, 0.0, 4.5, frame_id="/"+self.robot_name+"/base_link")

        return self.look_at_point(goal)

    def look_at_standing_person(self, timeout=0):
        """
        Gives a target at z = 1.75 at 1 m in front of the robot
        """
        goal = VectorStamped(1.0, 0.0, 1.6, frame_id="/" + self.robot_name + "/base_link")

        return self.look_at_point(goal)

    # -- Functionality --

    def look_at_point(self, vector_stamped, end_time=0, pan_vel=1.0, tilt_vel=1.0, timeout=0):
        assert isinstance(vector_stamped, VectorStamped)
        point_stamped = kdlVectorStampedToPointStamped(vector_stamped)
        self._setHeadReferenceGoal(0, pan_vel, tilt_vel, end_time, point_stamped, timeout=timeout)

    def cancel_goal(self):
        self._ac_head_ref_action.cancel_goal()
        self._goal = None
        self._at_setpoint = False

    def wait_for_motion_done(self, timeout=5.0):
        self._at_setpoint = False
        starttime = rospy.Time.now()
        if self._goal:
            while (rospy.Time.now() - starttime).to_sec() < timeout:
                if self._at_setpoint:
                    rospy.sleep(0.3)
                    return True
                else:
                    rospy.sleep(0.1)
        return False

    # ---- INTERFACING THE NODE ---

    def _setHeadReferenceGoal(self, goal_type, pan_vel, tilt_vel, end_time, point_stamped=PointStamped(), pan=0, tilt=0, timeout=0):
        self.cancel_goal()

        self._goal = HeadReferenceGoal()
        self._goal.goal_type = goal_type
        self._goal.priority = 0 # Executives get prio 1
        self._goal.pan_vel = pan_vel
        self._goal.tilt_vel = tilt_vel
        self._goal.target_point = point_stamped
        self._goal.pan = pan
        self._goal.tilt = tilt
        self._goal.end_time = end_time
        self._ac_head_ref_action.send_goal(self._goal, done_cb = self.__doneCallback, feedback_cb = self.__feedbackCallback)

        start = rospy.Time.now()
        if timeout != 0:
            print "Waiting for %d seconds to reach target ..."%timeout
            while (rospy.Time.now() - start) < rospy.Duration(timeout) and not self._at_setpoint:
                rospy.sleep(0.1)

    def __feedbackCallback(self, feedback):
        self._at_setpoint = feedback.at_setpoint

    def __doneCallback(self, terminal_state, result):
        self._goal = None
        self._at_setpoint = False

    def get_image(self, timeout=5):
        rospy.loginfo("getting one image...")

        global cv_image
        cv_image = None
        cv = Condition()

        def callback(data):
            global cv_image

            cv.acquire()
            cv_image = data
            cv.notify()
            cv.release()

        # subscriber = rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback)  # for test with tripod kinetic
        subscriber = rospy.Subscriber("/" + self.robot_name + "/top_kinect/rgb/image", Image, callback)  # for the robot

        cv.acquire()
        for i in range(timeout):
            if cv_image:
                break
            if rospy.is_shutdown():
                return
            cv.wait(timeout=1)
        else:
            raise Exception('no image received from %s' % subscriber.name)
        subscriber.unregister()
        image = cv_image
        cv.release()

        rospy.loginfo("got %d bytes of image data", len(image.data))
        return image

    def project_roi(self, roi, frame_id=None):
        """ Projects a region of interest of a depth image to a 3D Point. Hereto, a service is used

        :param roi: sensor_msgs/RegionOfInterest
        :param frame_id: if specified, the result is transformed into this frame id
        :return: VectorStamped object
        """
        response = self.project_rois(rois=[roi])
        point = response.points[0]

        # Convert to VectorStamped
        result = VectorStamped(x=point.x, y=point.y, z=point.z,
                               frame_id=response.header.frame_id)

        # If necessary, transform the point
        if frame_id is not None:
            print "Transforming roi to {}".format(frame_id)
            result = result.projectToFrame(frame_id=frame_id, tf_listener=self.tf_listener)

        # Return the result
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

    def _get_faces(self, image):
        r = self._recognize_srv(image=image)
        rospy.loginfo('found %d face(s) in the image', len(r.recognitions))
        return r

    def learn_person(self, name='operator'):
        HEIGHT_TRESHOLD = 88
        WIDTH_TRESHOLD = 88
        try:
            image = self.get_image()
        except:
            rospy.logerr("Cannot get image")
            return False

        recognitions = self._get_faces(image).recognitions
        recognitions = [r for r in recognitions if r.roi.height > HEIGHT_TRESHOLD and r.roi.height > WIDTH_TRESHOLD]
        rospy.loginfo('found %d valid face(s)', len(recognitions))

        if len(recognitions) != 1:
            return False

        recognition = recognitions[0]

        rospy.loginfo('annotating that face as %s', name)
        self._annotate_srv(image=image, annotations=[Annotation(label=name, roi=recognition.roi)])

        return True

    def detect_faces(self):
        image = self.get_image()
        return self._get_faces(image).recognitions

    def clear_face(self):
        rospy.loginfo('clearing all learned faces')
        self._clear_srv()

    def _get_persons(self, image):
        r = self._get_persons_srv(image=image)
        rospy.loginfo('found %d persons(s) in the image', len(r.detections))
        return r

    def detect_persons(self):
        image = self.get_image()
        return self._get_persons(image).detections

    def detect_waving_persons(self):
        persons = []
        for person in self.detect_persons():
            height = person.neck.y - person.nose.y
            roi = RegionOfInterest(x_offset=person.nose.x - 10, y_offset=person.nose.y, width=20, height=height)
            sides = []
            for side in ['left', 'right']:
                elbow = getattr(person, '%s_elbow' % side)
                wrist = getattr(person, '%s_wrist' % side)
                shoulder = getattr(person, '%s_shoulder' % side)

                if math.isnan(elbow.confidence) or elbow.confidence < 0.4 or\
                        math.isnan(wrist.confidence) or wrist.confidence < 0.4 or\
                        math.isnan(shoulder.confidence) or shoulder.confidence < 0.4:
                    continue

                dx = elbow.x - wrist.x
                dy = elbow.y - wrist.y
                rospy.loginfo('%s arm: dx=%f dy=%f', side, dx, dy)

                angle = math.atan2(dy, dx)
                angle = math.degrees(angle)

                rospy.loginfo('arm angle: %f', angle)

                if angle < 45 or angle > 180 - 45:
                    rospy.loginfo('skipping %s arm because its not pointing upwards', side)
                    continue

                if wrist.y < shoulder.y:
                    rospy.loginfo('skipping %s arm because its not above the shoulder', side)

                if dy < 50:
                    rospy.loginfo('skipping %s arm because its not big enough', side)

                if person not in persons:
                    sides.append(side)

            if sides:
                persons.append(WavingResult(side=sides, roi=roi))

        rospy.loginfo('found %d waving persons', len(persons))
        return persons

    def detect_persons_3d(self):
        """
        :return: [Skeleton]
        """
        width = 10 # px
        height = 10 # px

        persons = self.detect_persons()

        rois = []
        for person in persons:
            for slot in person.__slots__:
                detection = getattr(person, slot)
                rois.append(RegionOfInterest(x_offset=detection.x - width//2, y_offset=detection.y - height//2, width=width, height=height))

        points = self.project_rois(rois, frame_id='map').points

        i = 0
        skeletons = []
        for person in persons:
            bodyparts = {}
            for slot in person.__slots__:
                bodyparts[slot] = points[i]
                i += 1

            skeletons.append(Skeleton(bodyparts))

        return skeletons

    def visualize_skeletons(self, skeletons):
        raise NotImplementedError()

#######################################
    # # WORKS ONLY WITH amiddle-open (for open challenge rwc2015)
    # def take_snapshot(self, distance=10, timeout = 1.0):

    #     self.look_at_ground_in_front_of_robot(distance)
    #     rospy.sleep(timeout)
    #     rospy.loginfo("Taking snapshot")
    #     res = self.snapshot_srv()

    #     return res

#######################################


if __name__ == "__main__":
    rospy.init_node('amigo_head_executioner', anonymous=True)
    head = Head()
