#! /usr/bin/env python
import math
from collections import namedtuple

import rospy
from threading import Condition
from geometry_msgs.msg import PointStamped
from head_ref.msg import HeadReferenceAction, HeadReferenceGoal
from std_srvs.srv import Empty
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from image_recognition_msgs.srv import Annotate, Recognize, RecognizeResponse, GetPersons
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
        self.bodyparts = bodyparts
        self.links =  [ ('left_ear', 'left_eye'),
                        ('left_elbow', 'left_wrist'),
                        ('left_eye', 'nose'),
                        ('left_hip', 'neck'),
                        ('left_shoulder', 'left_elbow'),
                        ('nose','neck'),
                        ('right_ear', 'right_eye'),
                        ('right_elbow', 'right_wrist'),
                        ('right_eye', 'nose'),
                        ('right_hip', 'neck'),
                        ('right_shoulder', 'right_elbow'),
                        ('right_wrist', 'right_elbow'),
                        ('right_ankle', 'right_knee'),
                        ('left_ankle', 'left_knee'),
                        ('left_knee', 'left_hip'),]

    def __iter__(self):
        return self.bodyparts.__iter__()

    def __index__(self, value):
        return self.bodyparts.__index__(value)

    def __getitem__(self, key):
        return self.bodyparts.__getitem__(key)

    def items(self):
        return self.bodyparts.items()

    def __repr__(self):
        return '%s(%r)' % (self.__class__.__name__, self.bodyparts)

    def generate_links(self):
        for (a, b) in self.links:
            if a in self.bodyparts and b in self.bodyparts:
                yield self.bodyparts[a].point
                yield self.bodyparts[b].point
                rospy.loginfo("Add link {}".format((a, b)))
            else:
                rospy.logwarn("Not all bodyparts of link {} found".format((a, b)))



class Head(RobotPart):
    def __init__(self, robot_name, tf_listener):
        super(Head, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._ac_head_ref_action = self.create_simple_action_client("/"+robot_name+"/head_ref/action_server",
                                                                    HeadReferenceAction)
        self._camera_lazy_sub = None
        self._camera_cv = Condition()
        self._camera_last_image = None

        self._annotate_srv = self.create_service_client('/' + robot_name + '/face_recognition/annotate', Annotate)
        self._recognize_srv = self.create_service_client('/' + robot_name + '/face_recognition/recognize', Recognize)
        self._clear_srv = self.create_service_client('/' + robot_name + '/face_recognition/clear', Empty)
        # self._get_persons_srv = self.create_service_client('/detect_persons', GetPersons)
        self._get_persons_srv = self.create_service_client('/' + robot_name + '/person_detection/detect_persons', GetPersons)

        self._projection_srv = self.create_service_client('/' + robot_name + '/top_kinect/project_2d_to_3d',
                                                          Project2DTo3D)

        self._skeleton_pub = rospy.Publisher("skeleton_markers", MarkerArray, queue_size=100)

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

    def look_at_point(self, vector_stamped, end_time=0, pan_vel=1.0, tilt_vel=0.8, timeout=0):
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

    def _image_cb(self, image):
        self._camera_cv.acquire()
        self._camera_last_image = image
        self._camera_cv.notify()
        self._camera_cv.release()

    def get_image(self, timeout=5):
        # self._camera_lazy_sub
        # self._camera_cv
        # self._camera_last_image

        # import ipdb; ipdb.set_trace()

        # lazy subscribe to the kinect
        if not self._camera_lazy_sub:
            # for test with tripod kinect
            # self._camera_lazy_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self._image_cb)
            # for the robot
            rospy.loginfo("Creating subscriber")
            self._camera_lazy_sub = rospy.Subscriber("/" + self.robot_name + "/top_kinect/rgb/image", Image, self._image_cb)
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
        result = VectorStamped(x=response.point.x, y=response.point.y, z=response.point.z,
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
        try:
            r = self._recognize_srv(image=image)
            rospy.loginfo('found %d face(s) in the image', len(r.recognitions))
        except rospy.ServiceException as e:
            rospy.logerr(e.message)
            r = RecognizeResponse()
        return r

    def learn_person(self, name='operator'):
        HEIGHT_TRESHOLD = 88
        WIDTH_TRESHOLD = 88
        try:
            image = self.get_image()
        except:
            rospy.logerr("Cannot get image")
            return False

        raw_recognitions = self._get_faces(image).recognitions
        recognitions = [r for r in raw_recognitions if r.roi.height > HEIGHT_TRESHOLD and r.roi.height > WIDTH_TRESHOLD]
        rospy.loginfo('found %d valid face(s)', len(recognitions))

        if len(recognitions) != 1:
            rospy.loginfo("Too many faces: {}".format(len(recognitions)))
            return False

        recognition = recognitions[0]

        rospy.loginfo('annotating that face as %s', name)
        try:
            self._annotate_srv(image=image, annotations=[Annotation(label=name, roi=recognition.roi)])
        except rospy.ServiceException as e:
            rospy.logerr('annotate failed:', e)
            return False

        return True

    def detect_faces(self, stamp=False):
        image = self.get_image()
        if stamp:
            return self._get_faces(image).recognitions, image.header.stamp
        else:
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

        person_slot_rois = []
        for i, person in enumerate(persons):
            for slot in person.__slots__:
                detection = getattr(person, slot)
                if detection.x == 0 or detection.y == 0:
                    continue
                x_offset = max(0, detection.x - width // 2)
                y_offset = max(0, detection.y - height // 2)
                width = width
                height = height

                roi = RegionOfInterest(x_offset=x_offset, y_offset=y_offset, width=width, height=height)
                print((i, slot, roi))
                person_slot_rois.append((i, slot, roi))

        try:
            input = [r for _, _, r in person_slot_rois]
            rospy.loginfo('project input: %s', str(input))
            ps = self.project_rois(input).points
        except ValueError as e:
            rospy.loginfo('project_rois failed: %s', e)
            return []

        skeletons = [dict() for _ in range(len(persons))]
        for j, (person, slot, _) in enumerate(person_slot_rois):
            p = ps[j]
            if math.isnan(p.point.x) or math.isnan(p.point.y) or math.isnan(p.point.z):
                rospy.loginfo('skipping %s because of invalid projection to 3d', slot)
                continue
            rospy.loginfo('adding point to person {}: {} at slot {}'.format(person, p.point, slot).replace('\n', ' '))
            skeletons[person][slot] = p

        for skeleton in skeletons:
            zmin = float('inf')
            for slot, point in skeleton.items():
                if point.point.z < zmin:
                    zmin = point.point.z

            for slot, point in skeleton.items():
                point.point.z = zmin

        skeletons = [Skeleton(bodyparts) for bodyparts in skeletons]
        self.visualize_skeletons(skeletons)
        return skeletons

    def visualize_skeletons(self, skeletons):
        """
        Publish a MarkerArray for the given skeletons
        :param skeletons: [Skeleton]
        :return: MarkerArray
        """
        skeleton_markers = MarkerArray()
        import itertools
        colors = itertools.cycle([ColorRGBA(0, 0, 1, 1), ColorRGBA(1, 0, 0, 1), ColorRGBA(0, 1, 1, 1)])

        for index, skeleton in enumerate(skeletons):
            try:
                if skeleton.bodyparts:
                    skeleton_markers.markers += self.markers_for_skeleton(skeleton, index, sphere_color=colors.next(),line_color=colors.next())
                else:
                    rospy.logerr("No bodyparts at index {}".format(index))
            except Exception as e:
                rospy.logerr("Could not visualize skeleton {}: {}".format(skeleton, e))

        self._skeleton_pub.publish(skeleton_markers)

    def markers_for_skeleton(self, skeleton, index=0, sphere_color=None, line_color=None):
        sphere_color = ColorRGBA(1, 0, 0, 1) if not sphere_color else sphere_color
        line_color = ColorRGBA(0, 0, 1, 1) if not line_color else line_color

        joints_marker = Marker()
        joints_marker.id = index
        # joints_marker.lifetime = rospy.Duration(30)
        joints_marker.type = Marker.SPHERE_LIST
        joints_marker.scale.x, joints_marker.scale.y, joints_marker.scale.z = 0.05, 0.05, 0.05
        joints_marker.action = Marker.ADD
        joints_marker.ns = "skeleton_spheres"
        joints_marker.header.frame_id = skeleton.bodyparts.values()[0].header.frame_id  # Just take the first one
        joints_marker.header.stamp = rospy.Time.now()
        joints_marker.points = [joint_ps.point for joint_name, joint_ps in skeleton.items()]
        joints_marker.colors = [sphere_color for _, _ in skeleton.items()]
        rospy.loginfo("Added {} joints for skeleton {}".format(len(joints_marker.points), skeleton))

        links_marker = Marker()
        links_marker.id = index
        # links_marker.lifetime = rospy.Duration(30)
        links_marker.type = Marker.LINE_LIST
        links_marker.ns = "skeleton_lines"
        links_marker.scale.x = 0.02
        links_marker.action = Marker.ADD
        links_marker.header.frame_id = skeleton.bodyparts.values()[0].header.frame_id  # Just take the first one
        links_marker.header.stamp = rospy.Time.now()
        links_marker.points = list(skeleton.generate_links())
        links_marker.colors = [line_color for _ in links_marker.points]  # TODO: not in sync, whould iterate over pairs of points here

        rospy.loginfo("Added {} links for skeleton {}".format(len(links_marker.points)/2, skeleton))  # /2 because lines are pairs of points

        return [joints_marker, links_marker]

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
