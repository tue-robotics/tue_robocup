import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class YoloSegmentor:
    def __init__(self) -> None:
        model_path = "~/MEGA/developers/Donal/yolov8x-seg.pt"
        device = "cuda"
        self.model = YOLO(model_path).to(device)
        self.class_ids = [42, 43, 44]  # See the COCO dataset for class id to label info (table=60, person = 0)
        self.active = False

        self.publisher = rospy.Publisher('/hero/segmented_image', Image, queue_size=10)
        self.subscriber = rospy.Subscriber('/hero/head_rgbd_sensor/rgb/image_raw', Image, self.callback)

    def start(self):
        self.active = True

    def stop(self):
        self.active = False

    @staticmethod
    def detect(model, frame):
        results = model(frame)
        result = results[0]
        segmentation_contours_idx = [np.array(seg, dtype=np.int32) for seg in result.masks.xy]
        class_ids = np.array(result.boxes.cls.cpu(), dtype="int")
        return class_ids, segmentation_contours_idx

    def extract_table_segment(self, image, class_ids, segmentations):
        table_mask = np.zeros_like(image, dtype=np.uint8)
        for class_id, seg in zip(class_ids, segmentations):
            if class_id in self.class_ids:
                cv2.fillPoly(table_mask, [seg], 255)
        return table_mask

    def callback(self, data):
        #rospy.loginfo("got message")
        if not self.active:
            return
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        #rospy.loginfo("converted message")

        classes, segmentations = self.detect(self.model, cv_image)
        table_segment = self.extract_table_segment(cv_image, classes, segmentations)

        # Publish the table segment as a binary mask
        table_message = bridge.cv2_to_imgmsg(table_segment, encoding="passthrough")
        self.publisher.publish(table_message)


if __name__ == '__main__':
    rospy.init_node("cutlery_detector")
    ts = YoloSegmentor()
    ts.start()
    rospy.spin()
