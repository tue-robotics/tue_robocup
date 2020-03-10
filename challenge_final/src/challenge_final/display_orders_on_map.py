import os
import cv2
import numpy as np
from datetime import datetime
import traceback

# ROS
import rospy
import smach
import cv_bridge
import rospkg

# TU/e Robotics
from robot_skills import get_robot_from_argv


def color_map(N=256, normalized=False):
    """
    Generate an RGB color map of N different colors

    :param N : int amount of colors to generate
    :param normalized: bool indicating range of each channel: float32 in [0, 1] or int in [0, 255]
    :return: a numpy.array of shape (N, 3) with a row for each color and each row is [R,G,B]
    """

    def bitget(byteval, idx):
        return ((byteval & (1 << idx)) != 0)

    dtype = 'float32' if normalized else 'uint8'
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i + 1  # skip the first color (black)
        for j in range(8):
            r |= bitget(c, 0) << 7 - j
            g |= bitget(c, 1) << 7 - j
            b |= bitget(c, 2) << 7 - j
            c >>= 3

        cmap[i] = np.array([r, g, b])

    cmap = cmap / 255 if normalized else cmap
    return cmap


def shadow(img, text, org, fontFace, fontScale, color, thickness=None, lineType=None, bottomLeftOrigin=None):
    shadow_org = list(org)
    for offset in [[-1,-1], [-1, 1], [1, 1], [1, -1]]:
        shadow_org = [org[0]+offset[0],
                      org[1]+offset[1]]
        cv2.putText(img, text, shadow_org, fontFace, fontScale, color, thickness=None, lineType=None, bottomLeftOrigin=None)


class DisplayOrdersOnMap(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['detected_people'])
        self.robot = robot

    def execute(self, ud):
        try:
            floorplan = cv2.imread(
                os.path.join(rospkg.RosPack().get_path('challenge_final'), 'img/floorplan.png'))
            floorplan_height, floorplan_width, _ = floorplan.shape

            try:
                person_detection_clusters = ud['detected_people']
                assert isinstance(person_detection_clusters, list), "Person detection clusters is not a list"
                assert all([isinstance(cluster, dict) for cluster in person_detection_clusters]),\
                    "Not all clusters are dicts"
            except Exception as e:
                rospy.logerr("Cannot show people on floor plan: {}".format(e))
                person_detection_clusters = []

            bridge = cv_bridge.CvBridge()
            c_map = color_map(N=len(person_detection_clusters), normalized=True)
            for i, person_detection in enumerate(person_detection_clusters):
                image = bridge.imgmsg_to_cv2(person_detection['rgb'], "bgr8")
                roi = person_detection['person_detection'].face.roi
                roi_image = image[roi.y_offset:roi.y_offset + roi.height, roi.x_offset:roi.x_offset + roi.width]

                desired_height = 150
                height, width, channel = roi_image.shape
                ratio = float(height) / float(desired_height)
                calculated_width = int(float(width) / ratio)
                resized_roi_image = cv2.resize(roi_image, (calculated_width, desired_height))

                x = person_detection['map_ps'].point.x
                y = person_detection['map_ps'].point.y

                x_image_frame = 9.04 - x
                y_image_frame = 1.58 + y

                pixels_per_meter = 158

                px = int(pixels_per_meter * x_image_frame)
                py = int(pixels_per_meter * y_image_frame)

                cv2.circle(floorplan, (px, py), 3, (0, 0, 255), 5)

                px_image = 0
                py_image = 0

                try:
                    px_image = min(max(0, px - calculated_width / 2), floorplan_width - calculated_width - 1)
                    py_image = min(max(0, py - desired_height / 2), floorplan_height - desired_height - 1)

                    if px_image >= 0 and py_image >= 0:
                        # could not broadcast input array from shape (150,150,3) into shape (106,150,3)
                        floorplan[py_image:py_image + desired_height,
                        px_image:px_image + calculated_width] = resized_roi_image
                        cv2.rectangle(floorplan, (px_image, py_image),
                                      (px_image + calculated_width, py_image + desired_height),
                                      (c_map[i, 2] * 255, c_map[i, 1] * 255, c_map[i, 0] * 255), 10)
                    else:
                        rospy.logerr("bound error")
                except Exception as e:
                    rospy.logerr("Drawing image roi failed: {}".format(e))

                label = "female" if person_detection['person_detection'].gender else "male"
                label += ", " + str(person_detection['person_detection'].age)

                cv2.putText(img=floorplan, text=label,
                            org=(px_image, py_image + 20),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7,
                            color=(0, 0, 255), thickness=2,
                            lineType=cv2.LINE_AA)

                try:
                    cv2.putText(img=floorplan,
                                text=person_detection['selection'],
                                org=(px_image, py_image + 40),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=0.7,
                                color=(0, 255, 0),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                except KeyError as key_err:
                    rospy.logerr(key_err)

                # cv2.circle(floorplan, (px, py), 3, (0, 0, 255), 5)

            filename = os.path.expanduser('~/floorplan-{}.png'.format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S")))
            cv2.imwrite(filename, floorplan)
            rospy.loginfo('Dumped image to {}'.format(filename))

            if self.robot:
                self.robot.hmi.show_image(filename, 120)
            else:
                cv2.imshow('floorplan', floorplan)

            return "succeeded"
        except Exception as ex:
            rospy.logerr("Could not plot orders on map, sorry!")
            traceback.print_exc(file=sys.stdout)
            rospy.logerr(traceback.format_exc())
            return 'failed'


if __name__ == "__main__":

    rospy.init_node("test_get_orders")

    # Robot
    # _robot = get_robot_from_argv(index=1)
    _robot = None
    import sys
    import pickle
    import random
    ppl_dicts = pickle.load(open(sys.argv[2]))
    # ppl_dicts is a list of dicts {'rgb':sensor_msgs/Image, 'person_detection':..., 'map_ps':...}

    # Test data
    # ToDo: load Reins pickled file here
    user_data = smach.UserData()

    random.shuffle(ppl_dicts)
    user_data['detected_people'] = ppl_dicts[:4]
    user_data['detected_people'][0]['selection'] = None
    user_data['detected_people'][1]['selection'] = 'coke'
    # user_data['detected_people'][2]['selection'] = 'water'
    user_data['detected_people'][3]['selection'] = 'energy'

    sm = DisplayOrdersOnMap(robot=_robot)
    print(sm.execute(user_data))


