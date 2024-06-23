import rospy
import cv2
import numpy as np
import math
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class YoloSegmentor:
    def __init__(self) -> None:
        model_path = "~/MEGA/developers/Donal/yolov8x-seg.pt"
        device = "cuda"
        self.model = YOLO(model_path).to(device)
        self.class_ids = [42, 43, 44]  # See the COCO dataset for class id to label info (fork = 42, knife = 43, spoon = 44)

    def detect(self, frame):
        results = self.model(frame)
        global result # Make 'result' global such that in can be accessed in the LeastSquaresMethod class
        result = results[0]
        segmentation_contours_idx = [np.array(seg, dtype=np.int32) for seg in result.masks.xy]
        
        class_ids = np.array(result.boxes.cls.cpu(), dtype="int")
        

        # Collecting boxes and masks for the specified class IDs
        global filtered_masks
        cutlery_boxes = []
        filtered_masks = []
        for i, box in enumerate(result.boxes):
            if int(box.cls) in self.class_ids:
                cutlery_boxes.append(box)
                filtered_masks.append(result.masks.xy[i])   
    
        x_center = None
        y_center = None

        # Check if cutlery_boxes is empty
        if not cutlery_boxes:
            coordinates_box = None  # No bounding boxes found
        else:
            # Access the xywh attribute of the first box in cutlery_boxes
            coordinates_box_tensor = cutlery_boxes[0].xywh
            if coordinates_box_tensor.numel() == 0:
                coordinates_box = None
            else:
                coordinates_box = coordinates_box_tensor[0].tolist()
                x_center, y_center = coordinates_box[0], coordinates_box[1]  # Center coordinates of the bounding box


        print(f"x_center, y_center = {x_center, y_center}") #print center coordinates
        return class_ids, segmentation_contours_idx, x_center, y_center  # outputs an integer with the name of the detected object as well as a segmentation of the object's contour

    def extract_table_segment(self, image, class_ids, segmentations):
        table_mask = np.zeros_like(image, dtype=np.uint8)
        for class_id, seg in zip(class_ids, segmentations):
            if class_id in self.class_ids:
                cv2.fillPoly(table_mask, [seg], color=(255, 0, 255)) #If fork, knife or spoon a pink/purple mask will be created 
            else:
                cv2.fillPoly(table_mask, [seg], color=(255, 0, 0)) #If another object from the COCO dataset a red mask will be created
        return table_mask

    def process_image(self, cv_image):
        classes, segmentations, x_center, y_center = self.detect(cv_image)
        table_segment = self.extract_table_segment(cv_image, classes, segmentations)
        return table_segment, x_center, y_center


class LeastSquaresMethod:
    def __init__(self):
        self.intercept = 0
        self.slope = 0

    def fit(self, x, y):
        self.slope, self.intercept = self.calculate_slope_intercept(x, y)

    def calculate_slope_intercept(self, x, y):
        mean_x = np.mean(x)
        mean_y = np.mean(y)
        numerator = np.sum((x - mean_x) * (y - mean_y))

        min_x = int(min(x))
        max_x = int(max(x))

        height, width, channels = image.shape 
        #for nearly vertical cases:
        if max_x - min_x < (1/3 * width): 
            numerator = np.sum((y - mean_y) ** 2)
            denominator = np.sum((x - mean_x) * (y - mean_y))
            slope = numerator / denominator 
        #for all other orientations:    
        else:
            numerator = np.sum((x - mean_x) * (y - mean_y))
            denominator = np.sum((x - mean_x) ** 2)
            slope = numerator / denominator  
            self.slope = slope 
        intercept = mean_y - slope * mean_x
        return slope, intercept

    def predict(self, x):
        return self.slope * x + self.intercept
    
    def object_direction(self, x_center, y_center):
        perpendicular_slope = -1/self.slope
        perpendicular_intercept = y_center - perpendicular_slope * x_center

        inner_array = filtered_masks[0] 
        x = inner_array[:,0]
        y = inner_array[:,1]
        coordinates_upper = 0
        coordinates_lower = 0
        if 0 <= perpendicular_slope < math.inf:
            for i in range(len(x)):
                if y[i]<= perpendicular_slope*x[i] + perpendicular_intercept:
                    coordinates_upper += 1
                else:
                    coordinates_lower += 1
        if -math.inf < perpendicular_slope < 0:
            for i in range(len(x)):
                if y[i]>= perpendicular_slope*x[i] + perpendicular_intercept:
                    coordinates_upper += 1
                else:
                    coordinates_lower += 1
        print("Size outline upper half of the mask:", coordinates_upper)
        print("Size outline lower half of the mask:", coordinates_lower)

        if coordinates_upper >= coordinates_lower: 
            upwards = True
        elif coordinates_upper < coordinates_lower:
            upwards = False    
        self.upwards = upwards

        return upwards

if __name__ == '__main__':
    ts = YoloSegmentor()
    image = cv2.imread('fork.jpg')
    table_segment, x_center, y_center = ts.process_image(image)

    model = LeastSquaresMethod()

    inner_array = result.masks.xy[0] # result.masks.xy is an array in an array, so has to be unpacked first

    x = inner_array[:,0] # first column = x coordinates of all points of the segmentation mask
    y = inner_array[:,1] # second column = y coordinates of all points of the segmentation mask

    model.fit(x, y) #fit a line through all coordinates of the segmentation mask using the least squares method
    print(f"Calculated slope: {model.slope}")
    print(f"Calculated intercept: {model.intercept}")

    # Draw yellow dot at the center point 
    if x_center != None:
        cv2.circle(table_segment, (int(x_center), int(y_center)), 5, (0, 255, 255), -1)

    # Draw the line created by the least squares method in green, this is only for visualization
    min_x = int(min(x))
    max_x = int(max(x))
    
    # this should be x,y starting point of line, x,y end point of line
    cv2.line(table_segment, (min_x, int(model.predict(min_x))), (max_x, int(model.predict(max_x))), (0, 255, 0), 2)

    upwards = model.object_direction(x_center, y_center)
    print("Object direction:", "upwards" if upwards else "downwards")

    cv2.imshow('Table Segment', table_segment)
    cv2.waitKey(0)