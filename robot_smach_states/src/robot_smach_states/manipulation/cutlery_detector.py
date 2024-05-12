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
        self.class_ids = [42, 43, 44]  # See the COCO dataset for class id to label info (fork = 42, knife = 43, spoon = 44)
        self.active = False

        self.intercept = 0
        self.slope = 0

        self.publisher = rospy.Publisher('/hero/segmented_image', Image, queue_size=10)
        self.subscriber = rospy.Subscriber('/hero/hand_camera/image_raw', Image, self.callback)

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
        coordinates_box = result.boxes.xywh.tolist()[0]  # Obtains all coordinates of the bounding box.
    
        x_center, y_center = coordinates_box[0], coordinates_box[1]  # Obtain the center coordinates of the bounding box = center coordinates object
        print(f"x_center, y_center = {x_center, y_center}") #print center coordinates
        
        return class_ids, segmentation_contours_idx, result, x_center, y_center

    def extract_table_segment(self, image, class_ids, segmentation_contours_idx):
        table_segment = np.zeros_like(image, dtype=np.uint8)

        for class_id, seg in zip(class_ids, segmentation_contours_idx):
            if class_id in self.class_ids:
                cv2.fillPoly(table_segment, [seg], color=(255, 0, 255)) #If fork, knife or spoon a pink/purple mask will be created 
            else:
                cv2.fillPoly(table_segment, [seg], color=(255, 0, 0)) #If another object from the COCO dataset a red mask will be created
        
        return table_segment
    
    def visualize_center_point(self, cv_image, x_center, y_center, table_segment):
        cv2.circle(table_segment, (int(x_center), int(y_center)), 5, (0, 255, 255), -1) # Draw yellow dot at the center point 
        return table_segment

    def calculate_slope_intercept(self, result):
        inner_array = result.masks.xy[0] # result.masks.xy is an array in an array, so has to be unpacked first
        x = inner_array[:,0] # first column = x coordinates of all points of the segmentation mask
        y = inner_array[:,1] # second column = y coordinates of all points of the segmentation mask

        mean_x = np.mean(x)
        mean_y = np.mean(y)
        min_x = int(min(x))
        max_x = int(max(x))

        #for nearly vertical cases:
        if max_x - min_x <150: #LOGISCHE WAARDE GEVEN, MAXIMALE VERWACHTE DIKTE VAN BESTEK IN PIXELS
            numerator = np.sum((y - mean_y) ** 2)
            denominator = np.sum((x - mean_x) * (y - mean_y))
            slope = numerator / denominator 

        #for all other orientations:    
        else:
            numerator = np.sum((x - mean_x) * (y - mean_y))
            denominator = np.sum((x - mean_x) ** 2)
            slope = numerator / denominator   
        intercept = mean_y - slope * mean_x

        print(f"Calculated slope: {slope}")
        print(f"Calculated intercept: {intercept}")
        return slope, intercept, min_x, max_x

    def predict(self, slope, intercept, min_x, max_x):
        y_minx = self.slope * min_x + self.intercept #y-coordinate corresponding to min_x
        y_maxx = self.slope * max_x + self.intercept #y-coordinate corresponding to max_x
        rospy.loginfo("predict")
        return y_minx, y_maxx   

    def visualize_orientation(self, min_x, max_x, y_minx, y_maxx, table_segment):
        cv2.line(table_segment, (min_x, int(y_minx)), (max_x, int(y_maxx)), (0, 255, 0), 2) # Draw the line created by the least squares method in green
        return table_segment

    def callback(self, data):
        rospy.loginfo("Received image data")

        if not self.active:
            rospy.loginfo("Callback inactive")
            return
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        rospy.loginfo("Image converted")

        class_ids, segmentation_contours_idx, result, x_center, y_center = self.detect(self.model, cv_image)
        table_segment = self.extract_table_segment(cv_image, class_ids, segmentation_contours_idx)

        # Visualize center point
        table_segment_with_center = self.visualize_center_point(cv_image, x_center, y_center, table_segment)
        rospy.loginfo("Center point visualized")

        # Calculate slope and intercept
        slope, intercept, min_x, max_x = self.calculate_slope_intercept(result)
        rospy.loginfo("Slope and intercept calculated")

        y_minx, y_maxx = self.predict(slope, intercept, min_x, max_x)

        # Visualize orientation
        table_segment_with_orientation = self.visualize_orientation(min_x, max_x, y_minx, y_maxx, table_segment_with_center)
        rospy.loginfo("Orientation visualized")

        # Publish the table segment as a binary mask
        table_message = bridge.cv2_to_imgmsg(table_segment_with_orientation, encoding="passthrough")
        self.publisher.publish(table_message)

        rospy.loginfo("Segmented image with orientation published")


if __name__ == '__main__':
    rospy.init_node("cutlery_detector")
    ts = YoloSegmentor()
    ts.start()

    rospy.spin()
