#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <visualization_msgs/Marker.h>

#include <challenge_restaurant/GetNormalScore.h>

namespace enc = sensor_msgs::image_encodings;

void publishNpclVisualizationMarker(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pc,
                                    const ros::Publisher& pub, int id, const std::string& ns,
                                    const std_msgs::Header& header)
{
  visualization_msgs::Marker m;

  m.action =  visualization_msgs::Marker::ADD;

  m.type = visualization_msgs::Marker::LINE_LIST;
  m.id = id;
  m.ns = ns;
  m.pose.orientation.w = 1;

  m.header = header;

  m.scale.x = 0.01;

  m.color.a = 1;
  double length = 0.1;

  m.colors.resize(2 * pc->size());
  m.points.resize(2 * pc->size());
  for (unsigned int i = 0; i < pc->size(); ++i)
  {
    const pcl::PointNormal& p = pc->points[i];

    std_msgs::ColorRGBA color;
    color.r = fabs(p.normal_z);
    color.g = fabs(p.normal_z);
    color.b = fabs(p.normal_z);
    color.a = 1.0;

    m.points[2*i].x = p.x;
    m.points[2*i].y = p.y;
    m.points[2*i].z = p.z;

    m.points[2*i+1].x = p.x + length*p.normal_x;
    m.points[2*i+1].y = p.y + length*p.normal_y;
    m.points[2*i+1].z = p.z + length*p.normal_z;

    m.colors[2*i].a = 1.0;
    m.colors[2*i+1].a = 1.0;

    m.colors[2*i].r = color.r * 0.2;
    m.colors[2*i+1].r = color.r * 1.0;
    m.colors[2*i].g = color.g * 0.2;
    m.colors[2*i+1].g = color.g * 1.0;
    m.colors[2*i].b = color.b * 0.2;
    m.colors[2*i+1].b = color.b * 1.0;
  }

  pub.publish(m);
}

class GetNormalScoreNode
{
public:
  GetNormalScoreNode(double timeout, size_t normal_estimation_num_neighbours)
    : timeout_(timeout)
    , visualization_publisher_(nh_.advertise<visualization_msgs::Marker>("normal_visualization", 1))
    , tree_(new pcl::search::KdTree<pcl::PointXYZ> ())
    , normal_estimation_num_neighbours_(normal_estimation_num_neighbours)
  {
    ROS_INFO("GetNormalCountNode initialized");
    srv_ = nh_.advertiseService("get_normal_score", &GetNormalScoreNode::srvCallback, this);
  }

private:
  bool srvCallback(challenge_restaurant::GetNormalScoreRequest& req, challenge_restaurant::GetNormalScoreResponse& res)
  {
    const sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "points", nh_, ros::Duration(timeout_));
    if (!msg)
    {
      ROS_ERROR("No pointcloud message received within %.2f seconds", timeout_);
      return false;
    }

    // Convert the msg to a real cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    normal_estimation_.setInputCloud(cloud);
    normal_estimation_.setSearchMethod(tree_);

    // Output datasets
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    normal_estimation_.setKSearch(normal_estimation_num_neighbours_);

    // Compute the features
    normal_estimation_.compute(*cloud_normals);

    // Copy the points to the point_normals cloud
    pcl::copyPointCloud(*cloud, *cloud_normals);

    ROS_INFO("Computed %lu normals for pointcloud of size=%lu", cloud->size(), cloud_normals->size());
    publishNpclVisualizationMarker(cloud_normals, visualization_publisher_, 0, "normals", msg->header);

    if (cloud_normals->empty())
    {
      ROS_ERROR("Empty cloud");
      return false;
    }

    res.score = 0;
    for (auto p : cloud_normals->points)
    {
      res.score += fabs(p.normal_z);
    }
    return true;
  }

  ros::NodeHandle nh_;
  double timeout_;
  ros::Publisher visualization_publisher_;

  tf::TransformListener tf_listener_;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  size_t normal_estimation_num_neighbours_;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation_;

  ros::ServiceServer srv_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_normal_score_node");
  GetNormalScoreNode node(ros::param::param("~timeout", 1.0),
                          ros::param::param("~normal_estimation_num_neighbours", 10));
  ros::spin();
}
