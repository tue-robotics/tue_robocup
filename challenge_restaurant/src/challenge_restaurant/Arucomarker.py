import numpy as np
import cv2
import rospy
import os
from cv_bridge import CvBridge, CvBridgeError
class ArucoVector:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
def rvec_to_euler(R):
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)



def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()

    detector = cv2.aruco.ArucoDetector(cv2.aruco_dict, parameters)
    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    image_copy = frame.copy()

    nMarkers = len(corners)
    rvec = [None] * nMarkers
    tvec = [None] * nMarkers


    markerLength = 0.07
    objPoints = np.array([[-markerLength / 2, markerLength / 2, 0],
                          [markerLength / 2, markerLength / 2, 0],
                          [markerLength / 2, -markerLength / 2, 0],
                          [-markerLength / 2, -markerLength / 2, 0]], dtype=np.float32)

    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(image_copy, corners, ids)
        for i in range(nMarkers):
            retval, rvecs, tvecs = cv2.solvePnP(objPoints, corners[i], matrix_coefficients, distortion_coefficients)
            rvec[i] = rvecs
            tvec[i] = tvecs

            cv2.drawFrameAxes(image_copy, matrix_coefficients, distortion_coefficients, rvec[i], tvec[i], markerLength * 1.5)
            output_path = os.path.expanduser(
                '~/ros/noetic/system/src/challenge_restaurant/src/challenge_restaurant/output_image.jpg')
            cv2.imwrite(output_path, image_copy)
    else:
        rospy.logerr("No ArUco markers detected in the frame.")
        return None, None

    return rvec, tvec


aruco_type = "DICT_5X5_250"

intrinsic_camera = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
distortion = np.array((-0.43948, 0.18514, 0, 0))


def get_aruco_pos(self, aruco_type, intrinsic_camera, distortion):

    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

    try:
        image = self.perception.get_image()
    except Exception as e:
        rospy.logerr("Can't get image: {}".format(e))
        return False

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(image, "bgr8")  # Specify desired encoding
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return False

    rvec, tvec = pose_estimation(cv_image, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)

    if rvec is None or tvec is None:
        rospy.logerr("Pose estimation failed.")
        return False

    R, _ = cv2.Rodrigues(rvec[0])

    T = np.zeros((4, 4))
    T[:3, :3] = R
    T[:3, 3] = tvec[0].flatten()
    T[3, 3] = 1
    T_inv = np.linalg.inv(T)

    x, y, z = T_inv[:3, 3]

    roll, pitch, yaw = rvec_to_euler(T_inv[:3, :3])

    rospy.loginfo("ArUco marker position - x: {:.2f}, y: {:.2f}, z: {:.2f}".format(x, y, z))
    rospy.loginfo("ArUco marker orientation - roll: {:.2f}, pitch: {:.2f}, yaw: {:.2f}".format(roll, pitch, yaw))

    return ArucoVector(x, y, z, roll, pitch, yaw)
