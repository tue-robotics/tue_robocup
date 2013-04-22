#ifndef FOLLOW_ME_CARROT_PLANNER_H_
#define FOLLOW_ME_CARROT_PLANNER_H_
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

class CarrotPlanner
{

public:

    CarrotPlanner(const std::string& name);

    ~CarrotPlanner();

    void initialize(const std::string &name);

    bool MoveToGoal(geometry_msgs::PoseStamped &goal);

private:

    bool setGoal(geometry_msgs::PoseStamped& goal);

    bool computeVelocityCommand(geometry_msgs::Twist& cmd_vel);

    void setZeroVelocity(geometry_msgs::Twist& cmd_vel);

    double sign(double x){
        return x < 0.0 ? -1.0 : 1.0;
    }

    void determineDesiredVelocity(tf::Vector3 e_pos, double e_theta, const geometry_msgs::Twist& current_vel,
                                  double dt, geometry_msgs::Twist& cmd_vel);

    double determineReference(double error_x, double vel, double max_vel, double max_acc, double dt);

    bool isClearLine(const tf::Vector3& goal);

    void laserScanCallBack(sensor_msgs::LaserScan laser_scan);

    double calculateHeading(const tf::Vector3& goal);

    void publishCarrot(const tf::Vector3& carrot, ros::Publisher& pub);

    void publishCmdVel(const geometry_msgs::Twist& cmd_vel, ros::Publisher& pub);

    double MAX_VEL;
    double MAX_ACC;
    double MAX_VEL_THETA;
    double MAX_ACC_THETA;
    double MAX_LOCAL_GOAL_DISTANCE;
    double LOOKAHEAD;
    double XY_GOALREGION;
    double THETA_GOALREGION;
    double MAX_STRAFE_DISTANCE;

    double MAX_YAW_ERROR_DRIVING;
    double MAX_YAW_ERROR_STILL;
    double STILL_MAX_VEL;
    double STILL_MAX_VEL_SQ;

    double MAX_PATH_DISTANCE_SQ;

    double GAIN;

    double FOLLOW_DISTANCE;
    std::string TRACKING_FRAME;

    bool initialized_;

    tf::Vector3 goal_, e_pos_;

    // timestamp of last time cmd_vel was published
    double t_last_cmd_vel_;

    ros::Publisher carrot_pub_, cmd_vel_pub_;
    ros::Subscriber laser_scan_sub_;

    geometry_msgs::Twist last_cmd_vel_;

    sensor_msgs::LaserScan laser_scan_;

    double angle_;

    bool laser_data_ready_;

};

#endif
