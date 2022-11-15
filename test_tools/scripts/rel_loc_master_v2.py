#!/usr/bin/env python
import numpy as np
import math as mt
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from scipy.spatial.distance import pdist, squareform

import tf2_ros
from tf.transformations import euler_from_quaternion


class getData:
    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('ny_node', anonymous=True)

        self.laser_sub = rospy.Subscriber("/hero/base_laser/scan", LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber("hero/base/measurements", Odometry, self.odom_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher('euler_coordinates', MarkerArray, queue_size=10)
        # self.pub = rospy.Publisher('labels', MarkerArray, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # define message counter for combining message data.
        self.message_counter = 0

        rospy.loginfo("my_class init complete")
        self.progressedlaserdata = None
        self.relativelocalisation = None

    def laser_callback(self, data):
        # rospy.loginfo("got laser data")
        self.progressedlaserdata = ProgressScanData(data)
        size_scan = len(self.progressedlaserdata.scan_ranges)
        messages_combined = 5
        if not hasattr(self, 'combined_scans'):
            # first run, define combined_scan array size based on sensor data
            self.combined_scans = np.empty(shape=(messages_combined * size_scan, 3))
            for i in range(size_scan):
                self.combined_scans[i, 0] = self.progressedlaserdata.euler_points[i, 0]
                self.combined_scans[i, 1] = self.progressedlaserdata.euler_points[i, 1]
                self.combined_scans[i, 2] = self.progressedlaserdata.euler_points[i, 2]
        elif self.message_counter < 5:
            # combine scans euler coordinates
            n = self.message_counter
            for i in range(size_scan):
                self.combined_scans[n * size_scan + i, 0] = self.progressedlaserdata.euler_points[i, 0]
                self.combined_scans[n * size_scan + i, 1] = self.progressedlaserdata.euler_points[i, 1]
                self.combined_scans[n * size_scan + i, 2] = self.progressedlaserdata.euler_points[i, 2]
            self.message_counter += 1
        elif self.message_counter == 5:
            # enough scans found, perform relative localisation
            #rospy.loginfo("enough scans found, start relative localisation")
            self.relativelocalisation = performLocalisation(self.combined_scans, self.progressedlaserdata.scan_angle, self.progressedlaserdata.scan_angle_min, self.progressedlaserdata.scan_angle_max)
            self.message_counter = 0
            # empty combined_scans
            self.combined_scans = np.empty(shape=(messages_combined * size_scan, 3))
        else:
            rospy.loginfo("ERROR")
            print(self.message_counter)



    def odom_callback(self, data):
        # rospy.loginfo("got odom data")
        fill = None

    def pub_velocity(self):
        rospy.loginfo("my_method")
        twist = Twist()
        twist.linear.x = 0.1
        self.pub.publish(twist)

    def pub_euler_coordinates(self):
        if self.progressedlaserdata != None and self.relativelocalisation != None:
            #rospy.loginfo("Publish euler coordinates of points")
            pub_euler_points = MarkerArray()
            progressed_points = self.relativelocalisation.euler_points
            cluster = self.relativelocalisation.cluster_labels
            cluster_centers = self.relativelocalisation.cluster_centers
            features = self.relativelocalisation.distance_features

            # show points with color label based on cluster
            for i in range(len(progressed_points)):
                # create color based on label

                x_pts = progressed_points[i, 1]
                y_pts = progressed_points[i, 2]
                if cluster[i] == -1:
                    # if cluster is clutter, make red (appears as black in RVIZ)
                    r = 255
                    g = 0
                    b = 0
                else:
                    # each cluster has a different color in a range from blue to red
                    j = ((cluster[i]) * 255 / len(np.unique(cluster)))
                    r = round(mt.sin(0.024 * j + 0) * 127 + 128)
                    g = round(mt.sin(0.024 * j + 2) * 127 + 128)
                    b = round(mt.sin(0.024 * j + 4) * 127 + 128)
                # create marker for specific point
                marker = Marker()
                marker.id = 0
                marker.header.frame_id = "map"
                marker.type = 8
                point = Point()
                point.x = x_pts
                point.y = y_pts
                marker.points = [point]
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = 1.0
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = 0.03
                pub_euler_points.markers.append(marker)

            # show cluster centers
            for i in range(len(cluster_centers)):
                # create color markers for cluster centers
                x_pts = cluster_centers[i, 0]
                y_pts = cluster_centers[i, 1]
                # each cluster has a different color in a range from blue to red
                j = (i * 255 / len(cluster_centers))
                r = round(mt.sin(0.024 * j + 0) * 127 + 128)
                g = round(mt.sin(0.024 * j + 2) * 127 + 128)
                b = round(mt.sin(0.024 * j + 4) * 127 + 128)
                # create marker for center point of cluster
                marker = Marker()
                marker.id = i
                marker.header.frame_id = "map"
                marker.type = 8
                point = Point()
                point.x = x_pts
                point.y = y_pts
                marker.points = [point]
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = 1.0
                # larger icon when point is leg
                for j in range(len(features)):
                    if i in features:
                        marker.scale.x = 0.3
                        marker.scale.y = 0.3
                        marker.scale.z = 0.3
                    else:
                        marker.scale.x = 0.1
                        marker.scale.y = 0.1
                        marker.scale.z = 0.1
                pub_euler_points.markers.append(marker)

            self.pub.publish(pub_euler_points)
            # clear data
            self.progressedlaserdata = None
            self.relativelocalisation = None
        else:
            #rospy.loginfo("Waiting for laserdata...")
            pass

    def get_table_pose(self):
        # get pose of table (frame dinner_table) with respect to the robot (frame base_link)
        try:
            transform = self.tfBuffer.lookup_transform('base_link', 'dinner_table', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        quaternion_list = [transform.transform.rotation.x,
                           transform.transform.rotation.y,
                           transform.transform.rotation.z,
                           transform.transform.rotation.w]
        yaw = euler_from_quaternion(quaternion_list)[2]
        return [x, y, yaw]

    def run(self):
        rospy.loginfo("my_class run")
        r = rospy.Rate(10)
        # spin() simply keeps python from exiting until this node is stopped
        while not rospy.is_shutdown():
            self.pub_euler_coordinates()
            #example on how to use the get_table_pose method
            trans = self.get_table_pose()
            if trans is not None:
                x = trans[0]
                y = trans[1]
                yaw = trans[2]
                rospy.loginfo(f"pose is [x: {x}, y: {y}, yaw: {yaw}")
            r.sleep()


class ProgressScanData:

    # imports scan message and output progressed euler data
    # pre-progressing functions should be added here
    def __init__(self, scan_msg):
        # prepare data for progressing
        self.scan_ranges = np.asarray(scan_msg.ranges)
        self.scan_angle = float(scan_msg.angle_increment)
        self.scan_angle_min = float(scan_msg.angle_min)
        self.scan_angle_max = float(scan_msg.angle_max)

        # return data
        self.euler_points = ProgressScanData.polEul(self)

    # convert polar coordinates to euler coordinates
    def polEul(self):
        # create angles and ranges arrays
        angles = np.arange(self.scan_angle_min, self.scan_angle_max, self.scan_angle)
        ranges_with_nan = self.scan_ranges
        ranges = np.nan_to_num(ranges_with_nan)
        euler = np.empty(shape=(len(angles), 3), dtype='float')
        for x in range(len(angles)):
            # each cycle, convert specific polar point to euler
            euler[x, 0] = angles[x]
            euler[x, 1] = mt.sin(angles[x]) * ranges[x]
            euler[x, 2] = mt.cos(angles[x]) * ranges[x]
        # return array [angle, x, y]
        return euler


class performLocalisation:
    def __init__(self, com_scan, scan_angle, scan_angle_min, scan_angle_max):
        self.euler_points = com_scan
        self.scan_angle = scan_angle
        self.scan_angle_min = scan_angle_min
        self.scan_angle_max = scan_angle_max

        self.cluster_labels = performLocalisation.db_scan_clustering(self)
        self.cluster_centers = performLocalisation.cluster_centers(self)
        self.distance_features = performLocalisation.cluster_distances(self)

    def db_scan_clustering(self):
        epsilon = 0.1
        min_points = 6
        data = self.euler_points
        points = data[:, 1:3]
        # perform dbscan on point list with epsilon and minimum points of cluster
        clusters = DBSCAN(eps=epsilon, min_samples=min_points).fit(points)
        return clusters.labels_

    def cluster_centers(self):
        points = self.euler_points
        clusters = self.cluster_labels
        counts = np.unique(clusters, return_counts=True)
        cluster_centers = np.empty(shape=(len(counts[1]) - 1, 2), dtype='float')
        for i in range(len(counts[1]) - 1):
            cluster_points_x = 0.0
            cluster_points_y = 0.0
            k = 0
            for j in range(len(clusters)):
                if clusters[j] == i:
                    cluster_points_x += points[j, 1]
                    cluster_points_y += points[j, 2]
                    k += 1
                else:
                    pass
            cluster_centers[i, 0] = cluster_points_x / k
            cluster_centers[i, 1] = cluster_points_y / k
        return cluster_centers

    def cluster_distances(self):
        features_distances = [1.145]
        amount_features = len(features_distances)
        error = 0.02
        cluster_labels = self.cluster_labels
        cluster_size = np.unique(cluster_labels)

        dist = pdist(self.cluster_centers)
        cluster_list = []
        for k in range(amount_features):
            current_feature_distance = features_distances[k]
            index = np.where((abs(dist - current_feature_distance ) < error))
            # determine points out of condensed matrix (https://stackoverflow.com/questions/5323818/condensed-matrix-function-to-find-pairs)
            for x in range(len(index[0])):
                d = (1 + mt.sqrt(1 + 8 * len(dist))) // 2
                b = 1 - (2 * d)
                i = int((-b - mt.sqrt(b ** 2 - 8 * index[0][x])) // 2)
                j = int(index[0][x] + i * (b + i + 2) // 2 + 1)
                cluster_list.append(i)
                cluster_list.append(j)

        #print(squareform(dist))
        #print(index)
        #print(cluster_list)
        #print(self.cluster_centers)

        for i in reversed(range(len(cluster_list))):
            if cluster_size[cluster_list[i]] > 25:
                cluster_list.pop(i)


        return cluster_list

    # def feature_elimination(self):
    #     # filter out positives based on feature information (e.g. size)
    #     #self.distance_features
    #     cluster_centers_points = self.cluster_centers
    #     cluster_labels = self.cluster_labels
    #     feature_cluster_centers = self.distance_features


if __name__ == '__main__':
    mything = getData()
    mything.run()
