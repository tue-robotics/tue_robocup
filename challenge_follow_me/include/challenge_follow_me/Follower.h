#ifndef FOLLOWER_H
#define FOLLOWER_H

// STL
#include <string>
#include <vector>

// ROS
#include "ros/ros.h"

// ROS action client or actions
#include "amigo_head_ref/HeadRefAction.h"
#include <actionlib/client/simple_action_client.h>
#include "tue_move_base_msgs/MoveBaseAction.h"

// ROS msgs
#include "geometry_msgs/PoseStamped.h"

// ROS srvs
#include "tue_move_base_msgs/GetPath.h"

// Carrot planner
#include "tue_carrot_planner/carrot_planner.h"

// WIRE
#include <wire_interface/Client.h>
#include <wire_interface/PropertySet.h>

// Problib
#include "problib/pdfs/PDF.h"


class Follower
{
public:
    Follower();
    Follower(ros::NodeHandle& nh, std::string frame, bool map, bool demo = false);

    //! Interfaces to other modules
    bool reset();  // clear world model and start following
    bool reset(double left_right_margin);  // clear world model and start following
    bool start();  // start follower
    void pause();  // pause but keep on tracking
    void resume(); // resume following after being paused
    bool update(); // update robot reference position
    void stop();   // stop following

    bool getCurrentOperatorPosition(double& x, double& y, double& phi, std::string frame);
    void updateFollowDistance(double new_distance);
 
private:

    //! Robot must stop moving
    void freezeRobot();

    //! Let the robot talk
    void say(std::string sentence, bool block = false);

    //! Get operator position
    bool getPositionOperator(std::vector<wire::PropertySet>& objects, pbl::Gaussian& pos_operator);

    //! Find an operator
    bool findOperator(pbl::Gaussian& pos_operator);
    bool findOperatorFast(pbl::Gaussian& pos_operator);

    //! Set head pan and tilt
    bool setHeadPanTilt(double pan = 0.0, double tilt = -0.2, bool block = true);

    //! Move robot
    bool moveTowardsPosition(pbl::Gaussian& pos, double offset, bool block = false);

    //! Plan path
    bool findPath(geometry_msgs::PoseStamped& end_goal, tue_move_base_msgs::GetPath& srv_get_path);

    //! Transform pose stamped
    bool transformPoseStamped(geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out, std::string frame_out);

    //! Add orientation to pose stamped using an angle theta
    void addQuaternion(geometry_msgs::PoseStamped& ps, double theta);

    //! Get a world model position as a Gaussian
    bool getPositionGaussian(pbl::PDF pos, pbl::Gaussian& pos_gauss);

    //! RGB lights
    void setRGB(std::string color);


    ///////////////
    // VARIABLES //
    ///////////////

    //! The node handle
    ros::NodeHandle nh_;

    //! Voice
    ros::Publisher pub_speak_;
    ros::ServiceClient srv_speak_;

    //! Actuation robot
    actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>* ac_head_;           // Head action client
    CarrotPlanner* carrot_planner_;                                                   // Planner (only without map)
    actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* ac_move_base_; // Move base action client
    ros::ServiceClient srv_client_move_base_;                                         // Service client for paths

    //! Interaction with WIRE
    wire::Client* wire_client_;
    ros::ServiceClient reset_wire_srv_client_;

    //! Transformations
    tf::TransformListener* listener_;

    //! Toggle perception
    ros::ServiceClient pein_client_; // old
    ros::ServiceClient ppl_client_;  // new

    //! Color AMIGO
    ros::Publisher rgb_pub_;

    //! Settings
    std::string nav_frame_;            // Frame in which navigation goals are given
    bool use_map_;                     // With or without map
    double time_out_operator_lost_;    // How much time without updates before lost
    std::string wm_prop_operator_;     // Property used to get 'operator' from WIRE
    std::string wm_val_operator_;      // Value of the 'operator' property in WIRE
    double follow_distance_;           // Distance to keep to operator
    double max_distance_new_operator_; // Distance operator
    std::string robot_base_frame_;     // Base frame of the robot

    //! Mode
    enum Mode
    {
        ACTIVE,
        PAUSE,
        IDLE
    };
    Mode mode_;

    //! Find operator settings
    double TIME_WAIT_MAX;
    double DIST_LEFT_RIGHT;
    double DIST_MIN;
    double DIST_MAX;
    double MAX_2D_DISTANCE_TORSO_FACE;

    //! Administration
    double t_last_check_, t_no_meas_, t_last_print_;
    double operator_last_var_;
    bool demo_;
    bool first_time_;

};

#endif // FOLLOWER_H
