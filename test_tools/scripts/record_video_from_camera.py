#! /usr/bin/env python3

import argparse
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageConverter:
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
        else:
            if self.video_writer is None:
                rospy.loginfo(f"Got image on topic, creating video writer with {self.fps} fps")
                rows, cols, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                self.video_writer = cv2.VideoWriter(self.file_name, fourcc, self.fps, (cols, rows))

            self.video_writer.write(cv_image)

    def clean_shutdown(self):
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo(f"Saving video file {self.file_name}")
        else:
            rospy.loginfo(f"No video saved, no image received on topic")

    def __del__(self):
        self.clean_shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Record video from a camera image topic',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--topic", default="/hero/head_rgbd_sensor/rgb/image_raw",
                        help="image topic of type sensor_msgs/Image encoded in bgr8")
    parser.add_argument("--file", default="video.avi",
                        help="name of the video file to save to")
    args = parser.parse_args(rospy.myargv())

    image_topic = args.topic
    video_name = args.file

    rospy.init_node("save_video", anonymous=True)
    rospy.loginfo(f"Starting video recorder for topic {image_topic} saving to video file {video_name}")
    ic = ImageConverter(image_topic, video_name)
    rospy.on_shutdown(ic.clean_shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
