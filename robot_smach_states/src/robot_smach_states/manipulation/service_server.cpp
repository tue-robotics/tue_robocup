#include <vector>

#include "ros/ros.h"
//SIMULATION
//#include "opening_door/door_info.h"
//REALITY
#include "robot_smach_states/door_info.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "cb_base_navigation_msgs/LocalPlannerActionResult.h"
#include <cmath>

//tf2 librairy
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//message library
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/PointCloud2.h"

//pcl librairy
#include "pcl_ros/transforms.h"
#include "pcl/common/common.h"
#include <pcl_ros/point_cloud.h>
//pcl filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
//pcl segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//visualization
#include <visualization_msgs/Marker.h>


//size
#define BOUNDING_BOX_SIZE 0.5

class doorOpener {

    public:
        //ros variables
        ros::NodeHandle* nh;
        ros::ServiceServer service;
        ros::Publisher marker_pub;
        visualization_msgs::Marker marker;
        geometry_msgs::Vector3Stamped y_doorDirection_frame_sensor;
        bool find;

        //constructor
        doorOpener(ros::NodeHandle* nh_ptr): nh(nh_ptr) {
            //initialise service and publisher
            this -> service  = nh -> advertiseService("door_info", &doorOpener::doorInfo_callback, this);
            marker_pub = nh -> advertise<visualization_msgs::Marker>("visualization_marker", 10);
            }

        void set_y_direction(geometry_msgs::Vector3Stamped y_direction_frame_map) {
            tf2_ros::Buffer tf_buffer(ros::Duration(20));
            tf2_ros::TransformListener tfListener(tf_buffer);
            geometry_msgs::TransformStamped transformStamped_doorToSensor;
            //convertion of this message into the sensor frame
            //ROS_INFO("y_direction_frame_map = %f, %f, %f", y_direction_frame_map.vector.x, y_direction_frame_map.vector.y, y_direction_frame_map.vector.z);
            if (tf_buffer.canTransform("head_rgbd_sensor_rgb_frame", "door_inside", ros::Time(0), ros::Duration(1.0))) {
                try {
                transformStamped_doorToSensor = tf_buffer.lookupTransform("head_rgbd_sensor_rgb_frame", "door_inside", ros::Time(0), ros::Duration(1.0));
                tf2::doTransform(y_direction_frame_map, this -> y_doorDirection_frame_sensor, transformStamped_doorToSensor);
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                }
            }
            else {
            ROS_INFO("no transform possible");
            }
            //ROS_INFO("y_doorDirection_frame_sensor = %f, %f, %f", this -> y_doorDirection_frame_sensor.vector.x, this -> y_doorDirection_frame_sensor.vector.y, this -> y_doorDirection_frame_sensor.vector.z);
            if (abs(this -> y_doorDirection_frame_sensor.vector.x) < 0.2) this -> y_doorDirection_frame_sensor.vector.x = 0;
            if (abs(this -> y_doorDirection_frame_sensor.vector.y) < 0.2) this -> y_doorDirection_frame_sensor.vector.y = 0;
            if (abs(this -> y_doorDirection_frame_sensor.vector.z) < 0.2) this -> y_doorDirection_frame_sensor.vector.z = 0;
        }

        geometry_msgs::PointStamped write_marker(geometry_msgs::PointStamped handle_vv_location_frame_map) {
            this -> find = false;

            //get the mesage from depth_registered
            boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPointCloudMessage;
            sensor_msgs::PointCloud2 PointCloudMessage;
            //transform variable
            tf2_ros::Buffer tf_buffer(ros::Duration(20));
            tf2_ros::TransformListener tfListener(tf_buffer);

            //create a marker and a publisher for him
            // Set the frame ID and timestamp
            marker.header.stamp = ros::Time::now();

            // Set the namespace and ID for this marker
            marker.ns = "clustersMarker";
            marker.id = 10;

            // Set the marker type to cube list
            marker.type = visualization_msgs::Marker::CUBE_LIST;

            // Set the marker scale
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;

            // Set the pose of the marker
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.lifetime = ros::Duration();

            //get the message
            sharedPointCloudMessage = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hero/head_rgbd_sensor/depth_registered/rectified_points", *nh);
            if (sharedPointCloudMessage != NULL) {
                PointCloudMessage = *sharedPointCloudMessage;
            }
            else{
                ROS_INFO("No point cloud message received");

                return handle_vv_location_frame_map;
            }

            geometry_msgs::PointStamped handle_vv_location_frame_sensor;


            //test transform
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::TransformStamped transformStamped_sensorToHero;

            if (tf_buffer.canTransform( PointCloudMessage.header.frame_id, "map", ros::Time(0), ros::Duration(1.0))) {
                try {
                transformStamped = tf_buffer.lookupTransform(PointCloudMessage.header.frame_id, "map", ros::Time(0), ros::Duration(1.0));
                transformStamped_sensorToHero = tf_buffer.lookupTransform("hero/base_link", PointCloudMessage.header.frame_id , ros::Time(0), ros::Duration(1.0));
                tf2::doTransform(handle_vv_location_frame_map, handle_vv_location_frame_sensor, transformStamped);
                }
                catch (tf2::TransformException& ex) {
                    ROS_WARN("%s", ex.what());
                    ROS_INFO("pb in transformation");
                    return handle_vv_location_frame_map;
                }
            }
            else {
                ROS_INFO("can not transform");
                return handle_vv_location_frame_map;
            }


            // This crops the pointcloud to a bounding box of 25 cm around the original handle location
            double min_x = handle_vv_location_frame_sensor.point.x - (BOUNDING_BOX_SIZE/2.0);
            double min_y = handle_vv_location_frame_sensor.point.y - (BOUNDING_BOX_SIZE/2.0);
            double min_z = handle_vv_location_frame_sensor.point.z - (BOUNDING_BOX_SIZE/2.0);

            double max_x = handle_vv_location_frame_sensor.point.x + (BOUNDING_BOX_SIZE/2.0);
            double max_z = handle_vv_location_frame_sensor.point.z + (BOUNDING_BOX_SIZE/2.0);
            double max_y = handle_vv_location_frame_sensor.point.y + (BOUNDING_BOX_SIZE/2.0);

            //create the pointcloud that will receive the output after cropping the data
            pcl::PointCloud<pcl::PointXYZ>::Ptr PC_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::fromROSMsg(PointCloudMessage, *PC_ptr);

            pcl::PointCloud<pcl::PointXYZ>::Ptr PC_cropped_frame_sensor_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // pcl::PointXYZ is the type of point , ocl::PointCloud create a pointcloud from this type of point

            //create the crop variable
            pcl::PassThrough<pcl::PointXYZ> crop_filter;
            crop_filter.setInputCloud(PC_ptr);
            //we are going to crop according to x, y  and z.
            crop_filter.setFilterFieldName("x");
            crop_filter.setFilterLimits(min_x, max_x);
            crop_filter.setFilterFieldName("y");
            crop_filter.setFilterLimits(min_y, max_y);
            crop_filter.setFilterFieldName("z");
            crop_filter.setFilterLimits(min_z, max_z);
            crop_filter.filter(*PC_cropped_frame_sensor_ptr);


            // Creating the KdTree object for the search method of the extraction
            // a cluster is a group of points that are close to each other

            long nb_cluster = 100;
            std::vector<pcl::PointIndices> cluster_indices;

            while (nb_cluster > 10) {
                std::vector<pcl::PointIndices> cluster_result;
                PC_cropped_frame_sensor_ptr = this -> removePlaneSurface(PC_cropped_frame_sensor_ptr);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
                tree->setInputCloud(PC_cropped_frame_sensor_ptr);
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(0.01); // 1cm
                ec.setMinClusterSize(100);
                ec.setMaxClusterSize(25000);
                ec.setSearchMethod(tree);
                ec.setInputCloud(PC_cropped_frame_sensor_ptr);
                ec.extract(cluster_result);

                nb_cluster = cluster_result.size();
                if (nb_cluster <= 10){
                    cluster_indices = cluster_result;
                }
            }

            //check the position of every cluster to know which one is the closest the VV of the handle
            double i = 0; //count for marker color
            double max_error = 0.3; //to get the point that will be use to grab the handle
            pcl::PointCloud<pcl::PointXYZ>::Ptr handle_cluster = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); //cluster that will represent the handle
            std::vector<pcl::PointIndices> cluster_indices_selection; //dynamic array to store the index of the cluster that may represent the handle
            double min_y_frame_sensor = 15; //because we are going to choose the point that is closest to the sensor according to it y direction

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
                //create a new pointcloud for the cluster
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                //fill the pointcloud with the points of the cluster
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) cloud_cluster->points.push_back(PC_cropped_frame_sensor_ptr->points[*pit]);
                //fill the other info
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                //compute the center of the cluster
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud_cluster, centroid);

                //compute the difference between the center of the cluster and the handle
                double error_x = std::abs(centroid(0) - handle_vv_location_frame_sensor.point.x);
                double error_y = std::abs(centroid(1) - handle_vv_location_frame_sensor.point.y);
                double error_z = std::abs(centroid(2) - handle_vv_location_frame_sensor.point.z);
                double measured_error = error_x + error_y + error_z;

                geometry_msgs::Point p;
                p.x = centroid[0];
                p.y = centroid[1];
                p.z = centroid[2];

                std_msgs::ColorRGBA color;
                color.r = 1.0;
                color.a = 1.0;
                marker.points.push_back(p);
                marker.colors.push_back(color);


                if (measured_error < max_error){
                    //selection based of the distance between the VV and the center of the cluster has been made
                    //second criteria : according to the door, the point must be more in the middle than the VV
                    if (this -> y_doorDirection_frame_sensor.vector.x == 0) {
                        ROS_INFO("probleme in the direction");
                        return handle_vv_location_frame_map;
                    }

                    if (this -> y_doorDirection_frame_sensor.vector.x > 0) {
                        /*
                        all the value that are in y direction < 0 according to the frame door are outisde of the door. (y frame door is x frame sensor)
                        It means they can't represent the handle, so we must keep only the one that are superior to y of the VV
                        So we are going to check if the x value of the center of the cluster is superior to the x value of the VV
                        */
                        if (centroid(0) > handle_vv_location_frame_sensor.point.x - 0.05) { //3cm to remain safe
                            //the cluster is in the right direction
                            if (centroid(2) < min_y_frame_sensor) {
                                //the cluster is the closest to the sensor
                                min_y_frame_sensor = centroid(2);
                                *handle_cluster = *cloud_cluster;
                                this -> find = true;
                            }

                        }

                    }
                    else {
                        // same idea but on the other side of the door
                        //ROS_INFO("we are in the else, negative side");
                        if (centroid(0) < handle_vv_location_frame_sensor.point.x + 0.05) { //3 cm to remain safe
                            //the cluster is in the right direction
                            if (centroid(2) < min_y_frame_sensor) {
                                //the cluster is the closest to the sensor
                                min_y_frame_sensor = centroid(2);
                                *handle_cluster = *cloud_cluster;
                                this -> find = true;
                            }
                        }

                    }
                }

                i++;

            }

            marker.header.frame_id = PointCloudMessage.header.frame_id;

            //making the handle cluster cube bigger than the other
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*handle_cluster, centroid);
            geometry_msgs::Point p;
            p.x = centroid[0];
            p.y = centroid[1];
            p.z = centroid[2];

            std_msgs::ColorRGBA color;
            color.r = 0;
            color.a = 1.0;
            color.g = 1.0;
            marker.points.push_back(p);
            marker.colors.push_back(color);

            geometry_msgs::PointStamped handle_location_frame_sensor; //pointStamped that will be return
            handle_location_frame_sensor.header.frame_id = PointCloudMessage.header.frame_id;
            handle_location_frame_sensor.point.x = centroid[0];
            handle_location_frame_sensor.point.y = centroid[1];
            handle_location_frame_sensor.point.z = centroid[2];
            


            //transform the point from sensor frame to map frame
            geometry_msgs::PointStamped handle_location_frame_robot;
            tf2::doTransform(handle_location_frame_sensor, handle_location_frame_robot, transformStamped_sensorToHero);
            return handle_location_frame_robot;
            
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr removePlaneSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
            /*
            this fct remove 50% of the point cloud, it is used to remove the plane surface of the door
            */

            uint32_t nb_point = cloud_in -> points.size();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_intermediate = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            // Create the segmentation object for the planar model and set all the parameters
            pcl::SACSegmentation<pcl::PointXYZ> seg; //segmentation object
            pcl::PointIndices::Ptr inliers = pcl::make_shared<pcl::PointIndices>(); //inliers points of the shape
            pcl::ModelCoefficients::Ptr coefficients = pcl::make_shared<pcl::ModelCoefficients>(); //
            pcl::PCDWriter writer;

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(100);
            seg.setDistanceThreshold(0.01);

            while (cloud_in -> points.size() > 0.5 * nb_point) {
                //segment the largest planar of the cloud
                seg.setInputCloud(cloud_in);
                seg.segment(*inliers, *coefficients);

                if (inliers -> indices.size() == 0) {
                    ROS_INFO("Could not estimate a planar model for the given dataset.");
                    EXIT_FAILURE;
                }

                //extract the planar inliehandlers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_in);
                extract.setIndices(inliers);
                extract.setNegative(false);

                //get the points associated with the planar surface
                extract.filter(*cloud_intermediate);

                //remove the planar inliers, extract the rest
                extract.setNegative(true);
                extract.filter(*cloud_intermediate);

                *cloud_in = *cloud_intermediate;
            }

            return cloud_in;
        }

        void publish_marker(){
            ROS_INFO("publishing marker");
            ros::Rate r(1);
            while (ros::ok()) {
                ros::spinOnce();
                r.sleep();
                marker_pub.publish(marker);
                r.sleep();
                ros::spinOnce();
                r.sleep();
                ros::spinOnce();
            }
        }
        
        //REALITY
        bool doorInfo_callback(robot_smach_states::door_info::Request &msg_rqst, robot_smach_states::door_info::Response &msg_rsps) {
        //SIMULATION
        //bool doorInfo_callback(opening_door::door_info::Request &msg_rqst, opening_door::door_info::Response &msg_rsps) {
            ros::Rate sleeping_time(0.5);

            if (msg_rqst.input_string == "write_marker"){
                geometry_msgs::PointStamped p = this -> write_marker(msg_rqst.point_in);
                ROS_INFO("the point is (%f, %f, %f)", p.point.x, p.point.y, p.point.z);
                if (! this -> find) {
                    ROS_INFO("we didn't find the handle");
                    //the three point to zero.
                    p.point.x = 0;
                    p.point.y = 0;
                    p.point.z = 0;
                }
                msg_rsps.point_out = p;
                return true;
            }

            else if (msg_rqst.input_string == "publish_marker"){
                this -> publish_marker();
                return true;
            }

            else if (msg_rqst.input_string == "set_y_direction"){
                //we are receiving a pointStamped, we need to write a vector stamped
                geometry_msgs::Vector3Stamped y_direction;
                y_direction.header.frame_id = msg_rqst.point_in.header.frame_id;
                y_direction.vector.x = msg_rqst.point_in.point.x;
                y_direction.vector.y = msg_rqst.point_in.point.y;
                y_direction.vector.z = msg_rqst.point_in.point.z;
                this -> set_y_direction(y_direction);
                return true;
            }

            else {
                ROS_INFO("you sent a commande that does not exist");
                return false;
            }


        }

};


int main(int argc, char**argv) {
    ros::init(argc, argv, "door_informations");

    ros::NodeHandle nh;
    doorOpener opener = doorOpener(&nh);
    ros::spin();

    return 15;
}


