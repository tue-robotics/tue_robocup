import os
import pickle
from datetime import datetime

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge

from challenge_find_my_mates.cluster import cluster_people
from image_recognition_util.image_writer import color_map


class Face:
    def __init__(self, px, py, image):
        self.px = px
        self.py = py
        self._px = px
        self._py = py
        self.image = image
        self.image_height, self.image_width = image.shape[:2]

    def _overlaps_x(self, face):
        return abs(face.px - self.px) < self.image_width

    def _overlaps_y(self, face):
        return abs(face.py - self.py) < self.image_height / 2

    def correct(self, faces):
        for face in faces:
            if self._overlaps_x(face) and self._overlaps_y(face):
                if self._overlaps_x(face):
                    if self.px > face.px:
                        self.px = face.px + self.image_width
                    else:
                        self.px = face.px - self.image_width
                elif self._overlaps_y(face):
                    if self.py > face.py:
                        self.py = face.py + self.image_height
                    else:
                        self.py = face.py - self.image_height
                return True
        return False

    def draw(self, target_image, color):
        cv2.circle(target_image, (self._px, self._py), 10, (color[2] * 255, color[1] * 255, color[0] * 255), 5)
        cv2.line(target_image, (self._px, self._py), (self.px, self.py),
                 (color[2] * 255, color[1] * 255, color[0] * 255), 8)

        target_image_height, target_image_width = target_image.shape[:2]

        px_image = min(max(0, int(self.px - self.image_width / 2)), target_image_width - self.image_width - 1)
        py_image = min(max(0, int(self.py - self.image_height / 2)), target_image_height - self.image_height - 1)

        # Alter px_image and py_image to not overlap any other face

        if px_image >= 0 and py_image >= 0:
            # Could not broadcast input array from shape (150, 150, 3) into shape (106, 150, 3)
            target_image[py_image:py_image + self.image_height, px_image:px_image + self.image_width] = self.image
            cv2.rectangle(target_image, (px_image, py_image),
                          (px_image + self.image_width, py_image + self.image_height),
                          (color[2] * 255, color[1] * 255, color[0] * 255), 10)
        else:
            rospy.logerr("bound error")


def get_image(robot, room_id, person_detections):
    room_entity = robot.ed.get_entity(uuid=room_id)  # type: Entity
    if room_entity is None:
        msg = f"Could not find room: '{room_id}"
        rospy.logerr(msg)
        raise ValueError(msg)

    padding = -0.3

    def _get_clusters():
        in_room_detections = [d for d in person_detections if room_entity.in_volume(d['map_vs'], "in", padding)]

        rospy.loginfo("%d in room before clustering", len(in_room_detections))

        center_point = room_entity.pose.frame.p
        clusters = cluster_people(in_room_detections, np.array([center_point.x, center_point.y]))

        return clusters

    # filter in room and perform clustering until we have 4 options
    try:
        person_detection_clusters = _get_clusters()
    except ValueError as e:
        rospy.logerr(e)
        robot.speech.speak("Mates, where are you?", block=False)
        return "retry"

    for _ in range(3):
        floorplan = robot.ed.get_map([room_id])
        if floorplan is not None:
            break
    else:
        rospy.logerr("Could not fetch map robot.ed.get_map([room_id])")
        return "failed"
    floorplan_height, floorplan_width, _ = floorplan.map.shape

    bridge = CvBridge()
    c_map = color_map(n=len(person_detection_clusters), normalized=True)
    faces = []
    for i, person_detection in enumerate(person_detection_clusters):
        image = bridge.imgmsg_to_cv2(person_detection['rgb'], "bgr8")
        roi = person_detection['person_detection'].face.roi
        roi_image = image[roi.y_offset:roi.y_offset + roi.height, roi.x_offset:roi.x_offset + roi.width]

        desired_height = 120
        height, width, channel = roi_image.shape
        ratio = float(height) / float(desired_height)
        calculated_width = int(float(width) / ratio)
        resized_roi_image = cv2.resize(roi_image, (calculated_width, desired_height))

        vs_image_frame = floorplan.map_pose.frame.Inverse() * person_detection['map_vs'].vector

        px = int(vs_image_frame.x() * floorplan.pixels_per_meter_width)
        py = int(vs_image_frame.y() * floorplan.pixels_per_meter_height)

        try:
            face = Face(px, py, resized_roi_image)
            for _ in range(0, 10):  # Prevent inf loop
                if not face.correct(faces):
                    break
                rospy.loginfo("Correct face")
            face.draw(floorplan.map, c_map[i])
            faces.append(face)
        except Exception as e:
            rospy.logerr("Drawing image roi failed: {}".format(e))
            continue

        # label = "female" if person_detection['person_detection'].gender else "male"
        # label += ", " + str(person_detection['person_detection'].age)
        # cv2.putText(floorplan.map, label, (px_image, py_image + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),
        #             2, cv2.LINE_AA)

        # cv2.circle(floorplan, (px, py), 3, (0, 0, 255), 5)

    os.makedirs(os.path.expanduser('~/find_my_mates'), exist_ok=True)
    filename = os.path.expanduser(
        '~/find_my_mates/floorplan-{}.png'.format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S")))
    cv2.imwrite(filename, floorplan.map)
    rospy.loginfo(f"Wrote image to {filename}")

    return filename


if __name__ == '__main__':
    from robocup_knowledge import load_knowledge
    from robot_skills import get_robot
    import os.path
    import sys

    filename = os.path.expanduser(sys.argv[1])

    with open(filename, 'br') as f:
        ppl_dicts = pickle.load(f)

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot = get_robot("hero", 1)
    challenge_knowledge = load_knowledge('challenge_find_my_mates')
    get_image(robot, challenge_knowledge.room, ppl_dicts)
