#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError


def img_recognitions_to_rois(recognitions):
    """
    Extract the Region of Interest from a list of image recognitions

    :param recognitions: list of image recognitions
    :type recognitions: list[image_recognition_msgs/Recognition]
    :return: list of Regions of Interest
    :rtype: list[sensor_msgs/RegionOfInterest]
    """
    rois = []
    for recognition in recognitions:
        roi = recognition.roi
        rois.append(roi)
    return rois


def img_cutout(image, rois):
    """
    cutout the rois from the image

    :param image: image
    :type image: sensor_msgs/Image
    :param rois: Regions of Interest
    :type rois: list[sensor_msgs/RegionOfInterest]
    :return: Cutouts from the image
    :rtype: list[sensor_msgs/Image]
    """
    cvbridge = CvBridge()
    cutouts = []
    cv2_image = cvbridge.imgmsg_to_cv2(image, 'bgr8')
    for roi in rois:
        cv2_cutout = cv2_image[roi.y_offset:roi.y_offset + roi.height, roi.x_offset:roi.x_offset + roi.width]
        cutout = cvbridge.cv2_to_imgmsg(cv2_cutout, 'bgr8')
        cutouts.append(cutout)
    return cutouts


if __name__ == '__main__':
    import doctest
    doctest.testmod()
