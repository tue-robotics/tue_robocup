#!/usr/bin/env python
import time

import numpy as np
import math as mt
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from scipy.spatial.distance import pdist
import tf2_ros
from tf.transformations import euler_from_quaternion
from itertools import combinations

class getData:
    # communication with ROS can be found here

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('ny_node', anonymous=True)

        self.progressedlaserdata = None
        self.relativelocalisation = None
        self.choosedCandidate = None
        self.table_pose_x_self = None
        self.table_pose_y_self = None

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.laser_sub = rospy.Subscriber("/hero/base_laser/scan", LaserScan, self.laser_callback)
        #self.laser_sub = rospy.Subscriber("/hero/base_scan", LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber("hero/base/measurements", Odometry, self.odom_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher('euler_coordinates', MarkerArray, queue_size=2)
        # self.pub = rospy.Publisher('labels', MarkerArray, queue_size=10)
        rospy.loginfo("my_class init complete")

    def laser_callback(self, data):
        # perform localisation when laser data is received
        #rospy.loginfo("got scan data")

        # progress the laser data
        self.progressedlaserdata = ProgressScanData(data)

    def odom_callback(self, data):
        # rospy.loginfo("got odom data")
        fill = None

    def pub_velocity(self):
        rospy.loginfo("my_method")
        twist = Twist()
        twist.linear.x = 0.1
        self.pub.publish(twist)

    def pub_euler_coordinates(self):
        if self.relativelocalisation is not None and self.relativelocalisation.data_is_ready is True and self.choosedCandidate is not None:
            # create marker array for RVIZ, first delete old one
            pub_euler_points = MarkerArray()
            markerd = Marker()
            markerd.id = 0
            markerd.action = Marker.DELETEALL
            pub_euler_points.markers.append(markerd)

            cluster_centers = self.relativelocalisation.cluster_centers
            #features = self.relativelocalisation.filtered_results
            features = self.choosedCandidate.best_candidate[1:3]
            features.astype(int)
            print(features)

            # plot table
            marker = Marker()
            marker.id = 1
            marker.header.frame_id = "base_link"
            marker.type = 8
            point = Point()
            point.x = self.table_pose_x_self #+ 0.5 * 0.745 * mt.cos(self.table_pose_yaw_self) + 0.5 * 1.145 * mt.sin(
                #self.table_pose_yaw_self)
            point.y = self.table_pose_y_self #+ 0.5 * 1.145 * mt.sin(self.table_pose_yaw_self) + 0.5 * 0.75 * mt.cos(
                #self.table_pose_yaw_self)
            marker.points = [point]
            marker.color.r = 150
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            pub_euler_points.markers.append(marker)

            # marker = Marker()
            # marker.id = 2
            # marker.header.frame_id = "base_range_sensor_link"
            # marker.type = 8
            # point = Point()
            # point.x = self.table_pose_x_self - 0.5 * 0.745 * mt.cos(self.table_pose_yaw_self) - 0.5 * 1.145 * mt.sin(
            #     self.table_pose_yaw_self)
            # point.y = self.table_pose_y_self + 0.5 * 1.145 * mt.sin(self.table_pose_yaw_self) + 0.5 * 0.75 * mt.cos(
            #     self.table_pose_yaw_self)
            # marker.points = [point]
            # marker.color.r = 150
            # marker.color.g = 0
            # marker.color.b = 0
            # marker.color.a = 1.0
            # marker.scale.x = 0.3
            # marker.scale.y = 0.3
            # marker.scale.z = 0.3
            # pub_euler_points.markers.append(marker)
            #
            # marker = Marker()
            # marker.id = 3
            # marker.header.frame_id = "base_range_sensor_link"
            # marker.type = 8
            # point = Point()
            # point.x = self.table_pose_x_self + 0.5 * 0.745 * mt.cos(self.table_pose_yaw_self) - 0.5 * 1.145 * mt.sin(
            #     self.table_pose_yaw_self)
            # point.y = self.table_pose_y_self - 0.5 * 1.145 * mt.sin(self.table_pose_yaw_self) + 0.5 * 0.75 * mt.cos(
            #     self.table_pose_yaw_self)
            # marker.points = [point]
            # marker.color.r = 150
            # marker.color.g = 0
            # marker.color.b = 0
            # marker.color.a = 1.0
            # marker.scale.x = 0.3
            # marker.scale.y = 0.3
            # marker.scale.z = 0.3
            # pub_euler_points.markers.append(marker)
            #
            # marker = Marker()
            # marker.id = 4
            # marker.header.frame_id = "base_range_sensor_link"
            # marker.type = 8
            # point = Point()
            # point.x = self.table_pose_x_self - 0.5 * 0.745 * mt.cos(self.table_pose_yaw_self) - 0.5 * 1.145 * mt.sin(
            #     self.table_pose_yaw_self)
            # point.y = self.table_pose_y_self - 0.5 * 1.145 * mt.sin(self.table_pose_yaw_self) - 0.5 * 0.75 * mt.cos(
            #     self.table_pose_yaw_self)
            # marker.points = [point]
            # marker.color.r = 150
            # marker.color.g = 0
            # marker.color.b = 0
            # marker.color.a = 1.0
            # marker.scale.x = 0.3
            # marker.scale.y = 0.3
            # marker.scale.z = 0.3
            # pub_euler_points.markers.append(marker)

            # show bigger markers of cluster centers
            for k in range(len(cluster_centers)):
                x_pts = cluster_centers[k, 0]
                y_pts = cluster_centers[k, 1]

                # each cluster has a different color in a range from blue to red
                j = (k * 255 / len(cluster_centers))
                r = round(mt.sin(0.024 * j + 0) * 127 + 128)
                g = round(mt.sin(0.024 * j + 2) * 127 + 128)
                b = round(mt.sin(0.024 * j + 4) * 127 + 128)

                # create marker for the center point of the cluster
                marker = Marker()
                marker.id = 4+k # prevent from conflicting id from single point markers
                marker.header.frame_id = "base_range_sensor_link"
                marker.type = 8
                point = Point()
                point.x = x_pts
                point.y = y_pts
                marker.points = [point]
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = 1.0

                # the size of the marker is made bigger when the cluster is part of an identified feature
                for j in range(len(features)):
                    if k in features:
                        marker.scale.x = 0.2
                        marker.scale.y = 0.2
                        marker.scale.z = 0.2
                    else:
                        marker.scale.x = 0.1
                        marker.scale.y = 0.1
                        marker.scale.z = 0.1
                # add marker to marker array
                pub_euler_points.markers.append(marker)


            # publish marker array
            self.pub.publish(pub_euler_points)

            # clear data to wait for new scan data
            self.progressedlaserdata = None
            self.relativelocalisation = None
            self.choosedCandidate = None
        else:
            # if no new information is present, pass
            rospy.loginfo('Waitng for laser data...')
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
            # using worldmodel information
            trans = self.get_table_pose()
            if trans is not None:
                global table_pose_x, table_pose_y, table_pose_yaw
                table_pose_x = trans[0]
                table_pose_y = trans[1]
                table_pose_yaw = trans[2]
                self.table_pose_x_self = trans[0]
                self.table_pose_y_self = trans[1]
                self.table_pose_yaw_self = trans[2]
                if self.progressedlaserdata is not None:
                    # perform relative localisation
                    # this function can be altered for combining multiple scans
                    self.relativelocalisation = performLocalisation(self.progressedlaserdata.euler_points,
                                                                    self.progressedlaserdata.scan_angle,
                                                                    self.progressedlaserdata.scan_angle_min,
                                                                    self.progressedlaserdata.scan_angle_max)
                    # choose best candidate based on scores
                    self.choosedCandidate = candidateSystem(self.relativelocalisation.cluster_labels,
                                                            self.relativelocalisation.cluster_centers,
                                                            self.relativelocalisation.distance_features)
                    self.pub_euler_coordinates()
                    self.progressedlaserdata = None
                else:
                    pass

            r.sleep()


class ProgressScanData:
    # imports scan message and output progressed euler data
    # pre-progressing and filter functions should be added here

    def __init__(self, scan_msg):
        # prepare data for progressing
        self.scan_ranges = np.asarray(scan_msg.ranges)
        self.scan_angle = float(scan_msg.angle_increment)
        self.scan_angle_min = float(scan_msg.angle_min)
        self.scan_angle_max = float(scan_msg.angle_max)

        # return data
        self.euler_points_current_scan = ProgressScanData.polEul(self)
        self.euler_points = ProgressScanData.msg_data_buffer(self)

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
            euler[x, 2] = mt.sin(angles[x]) * ranges[x]
            euler[x, 1] = mt.cos(angles[x]) * ranges[x]
        # return array [angle, x, y]
        #rospy.loginfo("scan data progressed")
        return euler

    def msg_data_buffer(self):
        # amount of messages buffered
        max_buffer_size = 5

        # for first run, create data_buffer list
        if 'data_buffer' in globals():
            pass
        else:
            global data_buffer
            data_buffer = []
            print('Msg buffer created')

        #print('buffer size is', len(data_buffer))

        if len(data_buffer) >= 0 and len(data_buffer) < max_buffer_size:
            # is not full, add current data set in list
            data_buffer.append(self.euler_points_current_scan)
        elif len(data_buffer) == max_buffer_size:
            # buffer is full so remove latest data set before adding the current data set
            data_buffer.pop(0)
            data_buffer.append(self.euler_points_current_scan)
        else:
            rospy.loginfo("data buffer error")
            pass

        #print('new buffer size is', len(data_buffer))

        # return single array from buffer
        return np.concatenate(data_buffer, axis=0)


class performLocalisation:
    # relative localisation can be found here
    # clustering -> feature detection

    def __init__(self, com_scan, scan_angle, scan_angle_min, scan_angle_max):
        self.data_is_ready = False

        # collect all needed data
        self.euler_points = com_scan
        self.scan_angle = scan_angle
        self.scan_angle_min = scan_angle_min
        self.scan_angle_max = scan_angle_max

        # execute all functions
        # different functions can be added (or skipped)
        self.cluster_labels = performLocalisation.db_scan_clustering(self)
        self.cluster_centers = performLocalisation.cluster_centers(self)
        #print('clustering done')
        self.distance_features = performLocalisation.cluster_distances(self)
        self.filtered_results = performLocalisation.eliminate_false_positives(self)
        #print('localisation done')

        # activate publishing data
        self.data_is_ready = True

    def db_scan_clustering(self):
        # perform dbscan on point list with epsilon and minimum points of cluster
        epsilon = 0.1
        global data_buffer
        if len(data_buffer) > 2:
            min_points = len(data_buffer)
        else:
            min_points = len(data_buffer)+1
        data = self.euler_points
        points = data[:, 1:3]
        clusters = DBSCAN(eps=epsilon, min_samples=min_points).fit(points)
        return clusters.labels_

    def cluster_centers(self):
        # calculate cluster center of identified clusters
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
        #rospy.loginfo("clustering succesfull")
        return cluster_centers

    def cluster_distances(self):
        # find features based on distances between
        features_distances = [0.745, 1.145] #1.145 0.745
        amount_features = len(features_distances)
        error = 0.1
        cluster_labels = self.cluster_labels

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

        #for i in reversed(range(len(cluster_list))):
        #    # filter based on cluster size
        #    if cluster_size[cluster_list[i]] > 25:
        #        cluster_list.pop(i)

        return cluster_list

    def eliminate_false_positives(self):
        # eliminate false positives using world model coordinates
        # all positives further than a max_difference are dropped
        # the dropped positives could be similar object but not the object we are interested in

        center_to_legs = 0.681
        max_difference = 0.5
        feature_locations = self.cluster_centers[self.distance_features]
        #print("before filter")
        #print(feature_locations)
        feature_locations_filtered = None
        if feature_locations.shape[0] != 0:
            index = []
            for i in range(len(feature_locations)):
                difference_x = abs(table_pose_x - feature_locations[i][0])
                difference_y = abs(table_pose_y - feature_locations[i][1])
                difference = mt.sqrt(difference_x**2+difference_y**2)
                if difference <= (center_to_legs+max_difference):
                    index.append(i)
                else:
                    #print(difference)
                    pass
            # coordinates of features
            feature_locations_filtered = feature_locations[index]
            # cluster number of features
            feature_locations_filtered_cluster = [self.distance_features[i] for i in index]
            #feature_locations_filtered_cluster = self.distance_features[index]
        else:
            feature_locations_filtered = feature_locations
            feature_locations_filtered_cluster = []
            print("no features found")
            pass
        #print("after filter")
        #print(feature_locations_filtered)
        #print(index)
        #print(feature_locations_filtered_cluster)
        return feature_locations_filtered_cluster

class candidateSystem:
    # choose best result based on a candidate score system

    def __init__(self, cluster_labels, cluster_centers, feature_clusters):
        self.cluster_labels = cluster_labels
        self.cluster_counts = len(np.unique(cluster_labels)-1)
        self.cluster_centers = cluster_centers
        self.feature_clusters = feature_clusters

        self.cluster_combination_features = candidateSystem.candidate_pre_processing(self)
        self.worldmodel_scores = candidateSystem.candidate_worldmodel_features(self)
        self.angle_scores = candidateSystem.candidate_angle_features(self)

        # merge candidate points and print best candidate information
        for i in range(len(self.cluster_combination_features)):
            self.cluster_combination_features[i, 8] = self.worldmodel_scores[i] #+ self.angle_scores[i]
        candidates_sorted = self.cluster_combination_features[(-self.cluster_combination_features[:, 8]).argsort()]
        self.best_candidate = candidates_sorted[0, :]
        #print(self.best_candidate)
        print('best candidate is', self.best_candidate[1:3], 'with', self.best_candidate[8], 'points, on timestamp', rospy.Time.now())

    def candidate_pre_processing(self):
        # preprocess data in one array including candidate scores column
        # some additional filters can be added here (e.g. cluster size filtering)

        size_candidate_point_sets = int(len(self.feature_clusters)/2)
        cluster_combination_features = np.empty(shape=(size_candidate_point_sets, 9), dtype='float')
        j = 0
        for i in range(size_candidate_point_sets):
            # construct matrix, later: replace loop with matrix math
            cluster_combination_features[i, 0] = i  # index
            cluster_combination_features[i, 1] = self.feature_clusters[j] # cluster 1 label
            cluster_combination_features[i, 2] = self.feature_clusters[j+1] # cluster 2 label
            cluster_combination_features[i, 3] = self.cluster_centers[self.feature_clusters[j], 0] # x pos cluster 1
            cluster_combination_features[i, 4] = self.cluster_centers[self.feature_clusters[j], 1] # y pos cluster 1
            cluster_combination_features[i, 5] = self.cluster_centers[self.feature_clusters[j+1], 0] # x pos cluster 2
            cluster_combination_features[i, 6] = self.cluster_centers[self.feature_clusters[j+1], 1] # y pos cluster 2
            j += 2

        print(len(cluster_combination_features))
        return cluster_combination_features

    def candidate_worldmodel_features(self):
        # give candidates scores for distance of cluster combination to expected location

        # start with defining the expected location from worldmodel, middle point between legs
        global table_pose_x, table_pose_y, table_pose_yaw
        long_side_table = 1.14
        short_side_table = 0.745
        x_difference = 0.5*short_side_table
        y_difference = 0.5*long_side_table
        yaw = table_pose_yaw
        # calculate expected locations of middle point between legs
        expected_point_locations = np.array([[table_pose_x+x_difference*mt.cos(yaw), table_pose_y+y_difference*mt.sin(yaw)],
                                             [table_pose_x-x_difference*mt.sin(yaw), table_pose_y+y_difference*mt.cos(yaw)],
                                             [table_pose_x-x_difference*mt.cos(yaw), table_pose_y-y_difference*mt.sin(yaw)],
                                             [table_pose_x+x_difference-mt.sin(yaw), table_pose_y-y_difference*mt.cos(yaw)]])
        for i in range(len(self.cluster_combination_features)):
            # calculate coordinate of middle of a cluster combination
            x = (self.cluster_combination_features[i, 3]+self.cluster_combination_features[i, 5])/2 # mean x of candidate
            y = (self.cluster_combination_features[i, 4]+self.cluster_combination_features[i, 6])/2 # mean y of candidate
            diff_all = np.empty(shape=(len(expected_point_locations), 1), dtype='float')
            diff_candidate = np.empty(shape=(len(self.cluster_combination_features), 1), dtype='float')
            # calculate distances between expected and calculated middle points of cluster combinations
            for j in range(len(expected_point_locations)):
                diff_all_x = expected_point_locations[j][0] - x
                diff_all_y = expected_point_locations[j][1] - y
                diff_all[j] = mt.sqrt(diff_all_x**2 + diff_all_y**2)
            # choose smallest difference
            diff_all_sorted = np.sort(-diff_all, axis=0)
            diff_candidate[i] = diff_all_sorted[0]

        diff_candidate_sorted_index = sorted(range(len(diff_candidate)), key=lambda k: diff_candidate[k])
        #candidate_scores = np.array(range(len(self.cluster_combination_features)))
        #candidate_scores_sorted = candidate_scores[diff_candidate_sorted_index]
        candidate_scores_sorted = np.empty(shape=(len(self.cluster_combination_features), 1))

        for j in range(len(self.cluster_combination_features)):
            i = diff_candidate_sorted_index[j]
            candidate_scores_sorted[i] = len(self.cluster_combination_features) - j

        return candidate_scores_sorted

    def candidate_angle_features(self):
        # angle of features
        angle_threshold = 0.5*mt.pi
        angle_max_error = 0.1
        candidate_scores_angle = np.zeros(len(self.cluster_combination_features))

        # find shared cluster combinations
        comb_list_per_clus = np.empty((self.cluster_counts, ), dtype=object)

        for i in range(self.cluster_counts):
            # check for each cluster if it is found multiple times
            comb_list_per_cycle = []
            for j in range(len(self.cluster_combination_features)):
                if i == self.cluster_combination_features[j, 2] or i == self.cluster_combination_features[j, 3]:
                    # cluster i is present in cluster combination, note index of cluster combination
                    comb_list_per_cycle.append(j)
                else:
                    pass
            comb_list_per_clus[i] = comb_list_per_cycle

        for i in range(1, len(comb_list_per_clus)):
            if len(comb_list_per_clus[i]) > 1:
                # a cluster is part of multiple combinations
                # combined_cluster is the cluster for calculate the angle with other cluster
                combined_cluster = i

                # for each cluster combination where the combined cluster is part of
                other_clusters_x = []
                other_clusters_y = []
                index_candidate = []
                for k in range(len(comb_list_per_clus[i])):
                    index = comb_list_per_clus[i][k]
                    index_candidate.append(index)
                    if self.cluster_combination_features[index, 1] == combined_cluster:
                        # first of the two clusters of the combination is the combined cluster
                        combined_cluster_x = self.cluster_combination_features[index, 3]
                        combined_cluster_y = self.cluster_combination_features[index, 4]
                        other_clusters_x.append(self.cluster_combination_features[index, 5])
                        other_clusters_y.append(self.cluster_combination_features[index, 6])
                    elif self.cluster_combination_features[index, 2] == combined_cluster:
                        # second of the two clusters of the combination is the combined cluster
                        combined_cluster_x = self.cluster_combination_features[index, 5]
                        combined_cluster_y = self.cluster_combination_features[index, 6]
                        other_clusters_x.append(self.cluster_combination_features[index, 3])
                        other_clusters_y.append(self.cluster_combination_features[index, 4])
                    else:
                        rospy.loginfo('error in angle calculation')
                        print(combined_cluster)
                        break

                #for angle comparison, three points are needed. create all three point combinations
                other_clusters_indices = list(range(len(other_clusters_x)))
                angle_comb = list(combinations(other_clusters_indices, 2))
                for j in range(len(angle_comb)):
                    fir_clus_x = combined_cluster_x
                    fir_clus_y = combined_cluster_y
                    sec_clus_x = other_clusters_x[angle_comb[j][0]]
                    sec_clus_y = other_clusters_y[angle_comb[j][0]]
                    thir_clus_x = other_clusters_x[angle_comb[j][1]]
                    thir_clus_y = other_clusters_y[angle_comb[j][1]]

                    P12 = mt.sqrt((fir_clus_x - sec_clus_x) ** 2 + (fir_clus_y - sec_clus_y) ** 2)
                    P13 = mt.sqrt((fir_clus_x - thir_clus_x) ** 2 + (fir_clus_y - thir_clus_y) ** 2)
                    P23 = mt.sqrt((sec_clus_x - thir_clus_x) ** 2 + (sec_clus_y - thir_clus_y) ** 2)

                    angle = mt.acos((P12 ** 2 + P13 ** 2 - P23 ** 2) / (2 * P12 * P13))
                    #print(angle)

                    if abs(angle - angle_threshold) < angle_max_error:
                        for z in range(len(self.cluster_combination_features)):
                            if self.cluster_combination_features[z, 1] == combined_cluster or self.cluster_combination_features[z, 2] == combined_cluster:
                                candidate_scores_angle[z] += 10

        #print(candidate_scores_angle)
        return candidate_scores_angle



if __name__ == '__main__':
    mything = getData()
    mything.run()
