#include <ros/ros.h>
//#include <tue_move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
// Action client
//#include <actionlib/client/simple_action_client.h>
#include "tue_carrot_planner/carrot_planner.h"
#include <tf/tf.h>

using namespace std;

//string TRACKING_FRAME = "/base_link";

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me_simple");
    ros::NodeHandle nh;

    ROS_INFO("Initializing Carrot Planner");
    CarrotPlanner planner_ = CarrotPlanner("test_navigate");
    ROS_INFO("Initialized Carrot Planner");

    geometry_msgs::PoseStamped target;
    target.header.frame_id = "/base_link";
    target.pose.position.x = -0.01;
    target.pose.position.y = 1;
    target.pose.position.z = 0;

    tf::Quaternion q;
    q.setRPY(0, 0, atan2(target.pose.position.y, target.pose.position.x));

    //! Set orientation
    target.pose.orientation.x = q.getX();
    target.pose.orientation.y = q.getY();
    target.pose.orientation.z = q.getZ();
    target.pose.orientation.w = q.getW();

    ros::Rate follow_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        planner_.MoveToGoal(target);
        follow_rate.sleep();
    }

    return 0;
}
