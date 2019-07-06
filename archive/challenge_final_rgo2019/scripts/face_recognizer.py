#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Annotate, Recognize
from image_recognition_msgs.msg import Annotation


def filter_on_area(recognitions):
    """
    return the recognition with the biggest roi

    :param recognitions:
    :return:
    """
    best_roi_area = recognitions[0].roi.height * recognitions[0].roi.width
    best_rec = recognitions[0]
    for index, rec in enumerate(recognitions):
        roi_area = rec.roi.height * rec.roi.width
        rospy.logerr(roi_area)
        if roi_area > best_roi_area:
            best_rec = recognitions[index]
    return best_rec


class FaceRecognizer:
    def __init__(self):
        self._img_sub = rospy.Subscriber('image_to_ros', Image, self._image_cb)
        self._recognize_srv = rospy.ServiceProxy('face_recognition/recognize', Recognize)
        self._annotate_srv = rospy.ServiceProxy('face_recognition/annotate', Annotate)
        rospy.loginfo("Starting the FaceRecognizer")

    def _image_cb(self, image):
        self.learn_person(image=image)

    def learn_person(self, image, name='operator'):
        # HEIGHT_TRESHOLD = 88
        # WIDTH_TRESHOLD = 88

        try:
            recognitions = self._recognize_srv(image=image).recognitions
        except rospy.ServiceException as e:
            rospy.logerr('annotate failed: {}'.format(e))
            return False
        # raw_recognitions = self._recognize_srv(image=image).recognitions
        # recognitions = [r for r in raw_recognitions if r.roi.height > HEIGHT_TRESHOLD and r.roi.width > WIDTH_TRESHOLD]
        rospy.loginfo('found %d valid face(s)', len(recognitions))

        if len(recognitions) != 1:
            rospy.loginfo("Too many faces: {}".format(len(recognitions)))
            # return False

        if not recognitions:
            return False

        recognition = filter_on_area(recognitions)
        rospy.loginfo(recognition)

        rospy.loginfo('annotating that face as %s', name)
        try:
            self._annotate_srv(image=image, annotations=[Annotation(label=name, roi=recognition.roi)])
        except rospy.ServiceException as e:
            rospy.logerr('annotate failed: {}'.format(e))
            return False

        return True


if __name__ == '__main__':

    rospy.init_node("face_recognizer")

    node = FaceRecognizer()

    rospy.spin()
