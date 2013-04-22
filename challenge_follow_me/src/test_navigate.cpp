#include <ros/ros.h>
//#include <tue_move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
// Action client
//#include <actionlib/client/simple_action_client.h>
#include "challenge_follow_me/follow_me_carrot_planner.h"
#include <tf/tf.h>

using namespace std;

//string TRACKING_FRAME = "/base_link";

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me_simple");
    ros::NodeHandle nh;

    //double goal_x = 0.9;    // goal x in /base_link frame
    //double goal_y = 0.3;    // goal y in /base_link frame
    //double path_res = 0.1;  // maximum distance between waypoints in path

    //bool transform_to_map_frame = true;

    ///tf::TransformListener tf_listener;

    //actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction> move_base("move_base", true);
    //move_base.waitForServer();

    //tf_listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(10));

    //tue_move_base_msgs::MoveBaseGoal move_base_goal;

    ROS_INFO("Initializing Carrot Planner");
    CarrotPlanner* planner_;
    planner_ = new CarrotPlanner("test_navigate");
    ROS_INFO("Initialized Carrot Planner");

    geometry_msgs::PoseStamped target;
    target.header.frame_id = "/base_link";
    target.pose.position.x = -3;
    target.pose.position.y = -3;
    target.pose.position.z = 0;

    tf::Quaternion q;
    q.setRPY(0, 0, 1.57);

    //! Set orientation
    target.pose.orientation.x = q.getX();
    target.pose.orientation.y = q.getY();
    target.pose.orientation.z = q.getZ();
    target.pose.orientation.w = q.getW();

    ros::Rate follow_rate(10);
    while(ros::ok()) {
        planner_->MoveToGoal(target);
        follow_rate.sleep();
    }

    //planner_.MoveToGoal(target);
    //move_base_goal.path.push_back(target);

    //int nr_steps = (int)(max(abs((double)goal_x), abs((double)goal_y)) / path_res);

    //double dx = goal_x / nr_steps;
    //    double dy = goal_y / nr_steps;

    //    for(unsigned int i = 0; i < nr_steps; ++i) {
    //        geometry_msgs::PoseStamped waypoint;
    //        waypoint.header.frame_id = TRACKING_FRAME;
    //        waypoint.pose.position.x = i * dx;
    //        waypoint.pose.position.y = i * dy;
    //        waypoint.pose.position.z = 0;

    //        // set orientation
    //        // TODO: set correctly
    //        waypoint.pose.orientation.x = 0;
    //        waypoint.pose.orientation.y = 0;
    //        waypoint.pose.orientation.z = 0;
    //        waypoint.pose.orientation.w = 1;

    //        move_base_goal.path.push_back(waypoint);
    //    }

    //    geometry_msgs::PoseStamped goal;
    //    goal.header.frame_id = TRACKING_FRAME;
    //    goal.pose.position.x = goal_x;
    //    goal.pose.position.y = goal_y;
    //    goal.pose.position.z = 0;

    //    // set orientation
    //    // TODO: set correctly
    //    goal.pose.orientation.x = 0;
    //    goal.pose.orientation.y = 0;
    //    goal.pose.orientation.z = 0;
    //    goal.pose.orientation.w = 1;

    //    move_base_goal.path.push_back(goal);

    //    if (transform_to_map_frame) {
    //        for(unsigned int i = 0; i < move_base_goal.path.size(); ++i) {
    //            geometry_msgs::PoseStamped waypoint_transformed;
    //            tf_listener.transformPose("/map", move_base_goal.path[i], waypoint_transformed);
    //            move_base_goal.path[i] = waypoint_transformed;
    //        }
    //    }

    //    // send to move_base
    //    move_base.sendGoal(move_base_goal);

    return 0;
}
