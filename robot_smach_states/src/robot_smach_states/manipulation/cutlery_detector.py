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
        
        # Initialize the attributes to store the values
        self.x_center = None
        self.y_center = None
        self.slope = None
        self.upwards = None
        self.class_id = None
        self.time = rospy.Time.now()
        

    def start(self):
        self.active = True

    def stop(self):
        self.active = False

    @staticmethod
    def detect(model, frame):
        results = model(frame)
        global result
        result = results[0] 
        segmentation_contours_idx = [np.array(seg, dtype=np.int32) for seg in result.masks.xy]
        class_ids = np.array(result.boxes.cls.cpu(), dtype="int")

        return class_ids, segmentation_contours_idx, result

    def extract_table_segment(self, image, class_ids, segmentation_contours_idx):
        table_segment = np.zeros_like(image, dtype=np.uint8)
        global width
        height, width, channels = image.shape #obtain data on image size
        
        for class_id, seg in zip(class_ids, segmentation_contours_idx):
            self.class_id = class_id
            if class_id in self.class_ids:
                cv2.fillPoly(table_segment, [seg], color=(255, 0, 255)) #If fork, knife or spoon a pink/purple mask will be created 
        
        return table_segment
    
    def coordinates_center_point(self):
        coordinates_box = result.boxes.xywh.tolist()[0]  # Obtains all coordinates of the bounding box.
    
        x_center, y_center = coordinates_box[0], coordinates_box[1]  # Obtain the center coordinates of the bounding box = center coordinates object
        print(f"x_center, y_center = {x_center, y_center}") #print center coordinates
        
        # Store the values
        self.x_center = x_center
        self.y_center = y_center

        return x_center, y_center    
    
    def visualize_center_point(self, x_center, y_center, table_segment):
        cv2.circle(table_segment, (int(x_center), int(y_center)), 5, (0, 255, 255), -1) # Draw yellow dot at the center point 
        return table_segment

    def calculate_slope_intercept(self):
        inner_array = result.masks.xy[0] # result.masks.xy is an array in an array, so has to be unpacked first
        x = inner_array[:,0] # first column = x coordinates of all points of the segmentation mask
        y = inner_array[:,1] # second column = y coordinates of all points of the segmentation mask

        mean_x = np.mean(x)
        mean_y = np.mean(y)
        min_x = int(min(x))
        max_x = int(max(x))

        #for nearly vertical cases:
        if max_x - min_x < (1/5 * width): #if the object takes up a small width (less than 1/5 of the image) it is oriented (nearly) vertical in the camera coordinate frame
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
        
        
        self.time = rospy.Time.now()
        # Store the slope value
        self.slope = slope

        return slope, intercept, min_x, max_x

    def predict(self, slope, intercept, min_x, max_x):
        y_minx = slope * min_x + intercept #y-coordinate corresponding to min_x
        y_maxx = slope * max_x + intercept #y-coordinate corresponding to max_x
        rospy.loginfo("predict")
        return y_minx, y_maxx   

    def visualize_orientation(self, min_x, max_x, y_minx, y_maxx, table_segment):
        cv2.line(table_segment, (min_x, int(y_minx)), (max_x, int(y_maxx)), (0, 255, 0), 2) # Draw the line created by the least squares method in green
        return table_segment
    
    def object_direction(self, x_center):
        
        inner_array = result.masks.xy[0] 
        x = inner_array[:,0]
        coordinates_upper = 0
        coordinates_lower = 0
        for i in range(len(x)):
            xi = inner_array[i, 1]
            if xi >= x_center:
                coordinates_upper += 1
            elif xi < x_center:
                coordinates_lower += 1

        print("Size outline upper half of the mask:", coordinates_upper)
        print("Size outline lower half of the mask:", coordinates_lower)

        if coordinates_upper >= coordinates_lower: 
            upwards = True
        elif coordinates_upper < coordinates_lower:
            upwards = False    

        self.upwards = upwards

        return upwards

    def callback(self, data):
        #rospy.loginfo("Received image data")

        if not self.active:
            #rospy.loginfo("Callback inactive")
            return
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        rospy.loginfo("Image converted")

        class_ids, segmentation_contours_idx, result = self.detect(self.model, cv_image)
        rospy.loginfo("Object detected")

        table_segment = self.extract_table_segment(cv_image, class_ids, segmentation_contours_idx)

        #makes sure that even when no cutlery is detected, the segmentation of objects is visualized
        table_message = bridge.cv2_to_imgmsg(table_segment, encoding="passthrough")
        self.publisher.publish(table_message)

        if any(class_id in self.class_ids for class_id in class_ids):
            rospy.loginfo("Cutlery detected")

            x_center, y_center = self.coordinates_center_point()


            # Visualize center point
            table_segment_with_center = self.visualize_center_point(x_center, y_center, table_segment)
            rospy.loginfo("Center point visualized")

            # Calculate slope and intercept
            slope, intercept, min_x, max_x = self.calculate_slope_intercept()
            rospy.loginfo("Slope and intercept calculated")

            y_minx, y_maxx = self.predict(slope, intercept, min_x, max_x)

            # Visualize orientation
            table_segment_with_orientation = self.visualize_orientation(min_x, max_x, y_minx, y_maxx, table_segment_with_center)
            rospy.loginfo("Orientation visualized")

            # Publish the table segment as a binary mask
            table_message = bridge.cv2_to_imgmsg(table_segment_with_orientation, encoding="passthrough")
            self.publisher.publish(table_message)
            rospy.loginfo("Segmented image with orientation published")

            upwards = self.object_direction(x_center)
            rospy.loginfo("Direction determined")
        else:
            rospy.loginfo("No cutlery detected")

    # Method to be able to access values in the top_grasp code
    def data_class_id(self):
        return self.class_id

    def data_center_slope(self):
        return self.x_center, self.y_center, self.slope, self.time
    
    def data_direction(self):
        return self.upwards, self.time 


if __name__ == '__main__':
    rospy.init_node("cutlery_detector")
    ts = YoloSegmentor()
    ts.start()

    rospy.spin()