# ROS
import rospy
import smach
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvas
from visual_servoing_functions import convolution_full
import robot_skills.util.kdl_conversions as kdl_con
from tf.transformations import euler_from_quaternion

# TU/e Robotics
from ..manipulation.grasp_point_determination import GraspPointDeterminant
from robot_skills.util.entity import Entity
from ..util.designators import check_type
from geometry_msgs.msg import PointStamped


class GrabVisualServoing(smach.State):
    def __init__(self, robot, arm, grab_entity):
        """
        Pick up an item given an arm and an entity to be picked up
        :param robot: robot to execute this state with
        :param arm: Designator that resolves to the arm to grab the grab_entity with. E.g. UnoccupiedArmDesignator
        :param grab_entity: Designator that resolves to the entity to grab. e.g EntityByIdDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm
        self._camera_topic = "/hand_camera/image_raw"
        self._debug_image_pub = rospy.Publisher("image_debug_topic", Image)

        # Assign distance per movement (in y- and z-direction)
        self._move_disty = 0.03
        self._move_distz = 0.05

        self._bridge = CvBridge()

        check_type(grab_entity, Entity)
        self.grab_entity_designator = grab_entity
        self._gpd = GraspPointDeterminant(robot)

        # Kernel used for the edge_detection (higher value is darker color)
        self._edge_detect_kernel = np.array([[-1, -1, -1],
                                             [-1, 8, -1],
                                             [-1, -1, -1]])

    def execute(self, userdata=None):
        rospy.loginfo('Entering visual servoing state')

        grab_entity = self.grab_entity_designator.resolve()
        if not grab_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        rospy.loginfo('Starting visual servoing')

        # Finding the needed movement forward and maximum down, based on the first estimation
        foo = PointStamped()
        foo.header.frame_id = grab_entity.frame_id
        foo.point = kdl_con.kdl_frame_to_pose_msg(grab_entity._pose).position
        bar = self.robot.tf_listener.transformPoint("hero/grippoint_left", foo)
        rospy.loginfo("Entity pose w.r.t. gripper is {}".format(bar))
        est_x_1 = bar.point.x
        self.est_x = (est_x_1-0.05)/5
        self.est_y = bar.point.y
        self.est_z = bar.point.z
        rospy.loginfo('x-value is {}'.format(est_x_1))

        # First moving down, to ensure the object can be seen
        current_pose = self.robot.tf_listener.lookupTransform('/hero/base_link', 'hero/grippoint_left', rospy.Time())
        curr_roll, curr_pitch, curr_yaw = euler_from_quaternion(current_pose[1])
        curr_x, curr_y, curr_z = current_pose[0]
        next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(curr_x, curr_y, curr_z - 0.1,
                                                          curr_roll, curr_pitch, curr_yaw,
                                                          "/" + self.robot.robot_name + "/base_link")
        arm.send_goal(next_pose)
        arm.wait_for_motion_done()

        count = 0
        down = 0
        rospy.loginfo('Received first transform.')

        # Performing visual servoing in 5 steps
        while count < 5:
            current_pose = self.robot.tf_listener.lookupTransform('/hero/base_link', 'hero/grippoint_left', rospy.Time())
            curr_roll, curr_pitch, curr_yaw = euler_from_quaternion(current_pose[1])
            curr_x, curr_y, curr_z = current_pose[0]
            rospy.loginfo('Received transform from loop')
            move = self._visual_servo_feedback
            rospy.loginfo('Returned from magig function')

            # For each step is checked whether the movement needed is to the left or right and if downwards movement is needed
            if move[0] > 50 and move[1] > 10 and (self.est_z < -down * self._move_distz - 0.1):
                rospy.loginfo('Movement needed by the arm is {} pixels to the left and {} pixels down'.format(move[0], move[1]))
                next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(curr_x + self.est_x, curr_y + self._move_disty,  curr_z - self._move_distz,
                                                                  curr_roll, curr_pitch, curr_yaw,
                                                                  "/" + self.robot.robot_name + "/base_link")
                down += 1
            elif move[0] < -50 and move[1] > 10 and (self.est_z < -down * self._move_distz - 0.1):
                rospy.loginfo('Movement needed by the arm is {} pixels to the right and {} pixels down'.format(-1 * move[0], move[1]))
                next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(curr_x + self.est_x, curr_y - self._move_disty,  curr_z - self._move_distz,
                                                                  curr_roll, curr_pitch, curr_yaw,
                                                                  "/" + self.robot.robot_name + "/base_link")
                down += 1
            elif move[0] > 50 and move[1] < 10:
                rospy.loginfo('Movement needed by the arm is {} pixels to the left'.format(move[0]))
                next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(curr_x + self.est_x, curr_y + self._move_disty, curr_z,
                                                                  curr_roll, curr_pitch, curr_yaw,
                                                                  "/" + self.robot.robot_name + "/base_link")
            elif move[0] < -50 and move[1] < 10:
                rospy.loginfo('Movement needed by the arm is {} pixels to the right'.format(-1 * move[0]))
                next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(curr_x + self.est_x, curr_y - self._move_disty, curr_z,
                                                                  curr_roll, curr_pitch, curr_yaw,
                                                                  "/" + self.robot.robot_name + "/base_link")
            elif move[1] > 10 and (self.est_z < -down * self._move_distz - 0.1):
                rospy.loginfo('Movement needed by the arm is {} pixels down'.format(move[1]))
                next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(curr_x + self.est_x, curr_y, curr_z - self._move_distz,
                                                                      curr_roll, curr_pitch, curr_yaw,
                                                                      "/" + self.robot.robot_name + "/base_link")
                down += 1
            else:
                next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(curr_x + self.est_x, curr_y, curr_z,
                                                                      curr_roll, curr_pitch, curr_yaw,
                                                                      "/" + self.robot.robot_name + "/base_link")

            arm.send_goal(next_pose)
            arm.wait_for_motion_done()
            count += 1

        # Closing the gripper
        arm.send_gripper_goal('close')

        arm.occupied_by = grab_entity
        return 'succeeded'

    @property
    def _visual_servo_feedback(self):
        snap_image = rospy.wait_for_message(self._camera_topic, Image)
        rospy.loginfo('Received image')
        cv_img = self._bridge.imgmsg_to_cv2(snap_image, desired_encoding="passthrough")
        image = cv2.rotate(cv_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        rospy.loginfo('Converted image')
        # Cropping the image (removing top half)
        h, w, channels = image.shape
        image = image[h / 2:h, 0:w]

        # Applying color filters to the image and resizing
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.resize(image, (64, 48))

        # Mask of non-black pixels
        mask = image > 0

        # Coordinates of non-black pixels
        coords = np.argwhere(mask)

        # Bounding box of non-black pixels, slices are exclusive at the top
        x0, y0 = coords.min(axis=0)
        x1, y1 = coords.max(axis=0) + 1

        # Getting the contents of the bounding box
        image = image[x0:x1, y0:y1]
        image = image - image.mean(axis=0)
        image = image / np.abs(image).max(axis=0)

        # Saving and extracting the filtered image and canvas
        plt.rcParams['figure.dpi'] = 100
        fig, ax = plt.subplots(frameon=False)
        convolution_full(ax, image, self._edge_detect_kernel, vmin=-4, vmax=4, cmap='gray_r')
        fig.tight_layout()
        canvas = FigureCanvas(fig)
        canvas.draw()

        # Converting canvas to image
        graph_image = np.array(fig.canvas.get_renderer()._renderer)

        # Converting image to OpenCV's default bgr
        graph_image = cv2.cvtColor(graph_image, cv2.COLOR_BGR2GRAY)
        h, w = graph_image.shape
        graph_image = graph_image[30:h - 15, 30:w - 30]

        # Calculating the coordinates of the center of the image
        h1, w1 = graph_image.shape
        centerimg = [w1 / 2, h1 / 2]

        # Applying filters to blur the image (faster processing and less noise)
        font = cv2.FONT_HERSHEY_COMPLEX
        im1 = graph_image
        imCopy = im1.copy()
        kernel = np.ones((5, 5), np.float32) / 27
        im2 = cv2.filter2D(im1, -1, kernel)
        im = cv2.blur(im2, (5, 5))
        blurred_frame = cv2.GaussianBlur(im, (25, 25), 0)

        # Finding the contours in the image
        ret, thresh = cv2.threshold(blurred_frame, 127, 255, 0)
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Going through every contour found in the image and saving their deviations from the target position (center)
        deviations = []
        for cnt in contours:
            if cv2.contourArea(cnt) <= 500:
                continue

            approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)

            # Drawing boundaries of contours
            cv2.drawContours(imCopy, [approx], 0, (0, 0, 255), 1)

            # Calculating centers of the found contours
            xcoord = [x[0][0] for x in approx]
            ycoord = [x[0][1] for x in approx]
            centerx = np.mean(xcoord)
            centery = np.mean(ycoord)

            # Calculating distance to target position as deviations
            deviations.append((centerimg[0] - centerx, centerimg[1] - centery))

            # Flatten the array containing the co-ordinates of the vertices for plotting purposes
            n = approx.ravel()

            for i in range(len(n)):
                if i % 2 == 0:
                    x, y = n[i], n[i + 1]

                    # String containing the coordinates of the contours
                    string = str(x) + " " + str(y)

                    if i == 0:
                        # Plotting text on topmost coordinate
                        cv2.putText(imCopy, "Arrow tip", (x, y),
                                    font, 0.15, (255, 0, 0))
                    else:
                        # Plotting text on remaining coordinates.
                        cv2.putText(imCopy, string, (x, y),
                                    font, 0.15, (0, 255, 0))

        # Calculating the needed movement in pixels
        only_positive_y = [(x[0], x[1]) for x in deviations if x[1] > 1]
        move = np.argmin([np.abs(x[0]) for x in only_positive_y])

        # Sending the found needed movement to the while-loop
        return only_positive_y[move]


