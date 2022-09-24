#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class image_converter(object):
    def __init__(self, topic_name, file_name, fps=20):
        self.fps = fps
        self.file_name = file_name
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.callback)
        self.video_writer = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            raise
        else:
            if self.video_writer is None:
                rows, cols, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                self.video_writer = cv2.VideoWriter(self.file_name, fourcc, self.fps, (cols, rows))

            self.video_writer.write(cv_image)

    def clean_shutdown(self):
        if self.video_writer is not None:
            self.video_writer.release()
        rospy.loginfo(f"Saving video file {self.file_name}")

    def __del__(self):
        self.clean_shutdown()


def main():
    rospy.init_node("save_video", anonymous=True)
    ic = image_converter("/hero/head_rgbd_sensor/rgb/image_raw", "video.avi")
    rospy.on_shutdown(ic.clean_shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
