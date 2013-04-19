#include "challenge_follow_me/follow_me_carrot_planner.h"

CarrotPlanner::CarrotPlanner() : initialized_(false) {}

void CarrotPlanner::initialize(const std::string& name) {

    ros::NodeHandle private_nh("~/" + name);

    // parameters
    private_nh.param("max_vel_translation", MAX_VEL, 0.6);
    private_nh.param("max_acc_translation", MAX_ACC, 0.6);
    private_nh.param("max_vel_rotation", MAX_VEL_THETA, 0.6);
    private_nh.param("max_acc_rotation", MAX_ACC_THETA, 1.0);
    private_nh.param("xy_goal_tolerance", XY_GOALREGION, 0.1);
    private_nh.param("yaw_goal_tolerance", THETA_GOALREGION, 0.1);

    private_nh.param("max_yaw_error_driving", MAX_YAW_ERROR_DRIVING, 0.785398164); // 0.25 * PI
    private_nh.param("max_yaw_error_still", MAX_YAW_ERROR_STILL, 0.1);
    private_nh.param("still_max_vel", STILL_MAX_VEL, 0.01);

    private_nh.param("max_local_goal_distance", MAX_LOCAL_GOAL_DISTANCE, 0.5);
    private_nh.param("look_ahead", LOOKAHEAD, 2.0);
    private_nh.param("gain", GAIN, 0.9);

    private_nh.param("follow_distance", FOLLOW_DISTANCE, 2.0);

    STILL_MAX_VEL_SQ = STILL_MAX_VEL * STILL_MAX_VEL;

    carrot_pub_ = private_nh.advertise<visualization_msgs::Marker>("carrot", 1);
    vis_vel_pub_ = private_nh.advertise<visualization_msgs::Marker>("cmd_vel", 1);
    cmd_vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    laser_scan_sub_ = private_nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, &CarrotPlanner::laserScanCallBack, this);

    TRACKING_FRAME = "/base_link";

    initialized_ = true;

}

CarrotPlanner::~CarrotPlanner() {
}

bool CarrotPlanner::MoveToGoal(geometry_msgs::PoseStamped &goal){
    geometry_msgs::Twist cmd_vel;

    if (setGoal(goal)) {
        if(computeVelocityCommand(cmd_vel)) {
            cmd_vel_pub_.publish(cmd_vel);
            return true;
        }
    }
    return false;
}

bool CarrotPlanner::setGoal(geometry_msgs::PoseStamped &goal){
    if (goal.header.frame_id != TRACKING_FRAME){
        ROS_ERROR("Expecting goal in base_link frame");
        return false;
    }

    goal_.setX(goal.pose.position.x);
    goal_.setY(goal.pose.position.y);
    goal_.setZ(goal.pose.position.z);
    /// TODO: consdgvert to goal at FOLLOW_DISTANCE
    //double angle = calculateHeading(goal_);

    angle_ = tf::getYaw(goal.pose.orientation);


    //e_pos_.setX(cos(angle)*FOLLOW_DISTANCE);
    //e_pos_.setY(sin(angle)*FOLLOW_DISTANCE);
    //e_pos_.setZ(0.0);
    e_pos_ = goal_;

    publishCarrot(e_pos_, carrot_pub_);
    return true;

}

bool CarrotPlanner::computeVelocityCommand(geometry_msgs::Twist &cmd_vel){
    if(!initialized_){
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    // determine dt since last callback
    double time = ros::Time::now().toSec() + ros::Time::now().toNSec() / 1e9;
    double dt = 0;
    if (t_last_cmd_vel_ > 0) {
        dt = time - t_last_cmd_vel_;
    }
    t_last_cmd_vel_ = time;


    if(!isClearLine(e_pos_)) {
        setZeroVelocity(cmd_vel);
        return false;
    }

    // determine the error in the orientation
    double e_theta = 0;
    //e_theta = calculateHeading(goal_);
    e_theta = angle_;

    geometry_msgs::Twist e_pos_norm_msg;
    tf::Vector3 e_pos_norm;
    e_pos_norm = e_pos_.normalized();
    tf::vector3TFToMsg(e_pos_norm, e_pos_norm_msg.linear);

    ROS_DEBUG_STREAM("e_pos = " << e_pos_norm_msg.linear << ", e_theta = " << e_theta);

    determineDesiredVelocity(e_pos_, e_theta, last_cmd_vel_, dt, cmd_vel);

    last_cmd_vel_ = cmd_vel;

    return true;
}

bool CarrotPlanner::isClearLine(const tf::Vector3 &goal){
    /// TODO: check laser data inbetween target and robot
    //double angle = calculateHeading(goal_);
    int num_readings = laser_scan_.ranges.size();
    int num_incr = angle_/laser_scan_.angle_increment;
    double dist_to_obstacle = laser_scan_.ranges[num_readings/2 + num_incr];

    if (dist_to_obstacle < goal_.length() - 0.1) {
        ROS_WARN_STREAM("obstacle detected at " << dist_to_obstacle << ", check range is "  << goal_.length() - 0.1);
        return false;
    }

    return true;
}

void CarrotPlanner::laserScanCallBack(sensor_msgs::LaserScan laser_scan){
    if(laser_scan.header.frame_id == "/front_laser"){
        laser_scan_ = laser_scan;
    }
}


double CarrotPlanner::calculateHeading(const tf::Vector3 &goal) {
    return atan2(goal.getY(), goal.getX());
}

void CarrotPlanner::determineDesiredVelocity(tf::Vector3 e_pos, double e_theta, const geometry_msgs::Twist &current_vel,
                                             double dt, geometry_msgs::Twist &cmd_vel) {

    double e_norm = e_pos.length();

    tf::Vector3 current_vel_trans;
    tf::vector3MsgToTF(current_vel.linear, current_vel_trans);
    double v_norm_sq = current_vel_trans.length2();

    // determine the maximum error of the orientation,
    // if this error is reached the robot stops and only controls the rotation

    // during execution of the path
    double max_theta_error = MAX_YAW_ERROR_DRIVING;

    if (v_norm_sq < STILL_MAX_VEL_SQ) {
        // when the robot is standing still
        max_theta_error = MAX_YAW_ERROR_STILL;
    }

    double v_wanted_norm = 0;


    if ((fabs(e_theta) < max_theta_error && e_norm > 0)) {

        v_wanted_norm = std::min(MAX_VEL, GAIN * sqrt(2 * e_norm * MAX_ACC));
    }

    // make sure the wanted velocity has the direction towards the goal and the magnitude of v_wanted_norm
    tf::Vector3 vel_wanted;
    vel_wanted = e_pos.normalized();
    vel_wanted *= v_wanted_norm;

    // check if the acceleration bound is violated
    tf::Vector3 vel_diff = vel_wanted - current_vel_trans;
    double acc_wanted = vel_diff.length() / dt;
    if (acc_wanted > MAX_ACC) {
        tf::vector3TFToMsg(current_vel_trans + vel_diff.normalized() * MAX_ACC * dt, cmd_vel.linear);
    } else {
        tf::vector3TFToMsg(vel_wanted, cmd_vel.linear);
    }

    // the rotation is always controlled
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = determineReference(e_theta, current_vel.angular.z, MAX_VEL_THETA, MAX_ACC_THETA, dt);
}

// PARTLY TAKEN FROM amigo_ref_interpolator
double CarrotPlanner::determineReference(double error_x, double vel, double max_vel, double max_acc, double dt) {
    double EPS = 0.5 * max_acc*dt;

    //initial state
    bool still = false;
    bool move = false;
    bool dec = false;
    bool con = false;
    bool acc = false;

    double a = 0.0;
    double vel_mag = fabs(vel);

    //compute deceleration distance
    double delta_t1=vel_mag/max_acc; //deceleration segment time
    double dec_dist = 0.5*max_acc * (delta_t1) * (delta_t1); //deceleration distance

    //determine magnitude and sign of error vector
    double delta_x = fabs(error_x);
    int sign_x = sign(vel);

    //decide whether to move or stand still
    if (vel_mag!=0.0 || delta_x > EPS){
        move = true;
    } else {
        still = true;
        error_x = 0;
    }
    double dir = sign(error_x);

    //move: decide whether to stop, decelerate, constant speed or accelerate
    if (move){
        //		if (stopping){
        //			acc = false;
        //			con = false;
        //			still = false;
        //			dec = true;
        //			///ROS_ERROR("stopping");
        //       	} else
        if (fabs(dec_dist) >= fabs(delta_x)){
            dec = true;
            // ROS_INFO("go to dec");
        }
        else if (sign_x * error_x < 0 && vel_mag != 0.0){
            dec = true;
            // ROS_INFO("setpoint behind");
        }
        else if (fabs(dec_dist) < fabs(delta_x) && vel_mag >= max_vel){
            con = true;
            // ROS_INFO("go to con");
        }
        else {
            acc = true;
            // ROS_INFO("go to acc");
        }
        //move: reference value computations
        if (acc){
            vel_mag += max_acc * dt;
            vel_mag = std::min<double>(vel_mag, max_vel);
            //x+= dir * vel_mag * dt;
            a = dir * max_acc;
        }
        if (con){
            //x+= dir * vel_mag * dt;
            a = 0.0;
        }
        if (dec){
            vel_mag -= max_acc * dt;
            vel_mag = std::max<double>(vel_mag, 0.0);
            //x+= dir * vel_mag * dt;
            a = - dir * max_acc;
            if (vel_mag < (0.5 * max_acc * dt)){
                vel_mag = 0.0;
                //reset = true;
                ///ROS_WARN("reset");
            }
        }
        //ready = false;
    }

    //stand still: reset values
    else if (still){
        vel = 0;
        a = 0.0;
        sign_x = 0;
        //reset = true;
        //ready = true;
    }
    else {
    }

    vel = dir * vel_mag;
    return vel;
}

void CarrotPlanner::setZeroVelocity(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
}

void CarrotPlanner::publishCarrot(const tf::Vector3& carrot, ros::Publisher& pub) {
    /// create a message for the plan
    visualization_msgs::Marker marker;
    marker.header.frame_id = TRACKING_FRAME;
    marker.header.stamp = ros::Time::now();
    marker.ns = "carrot";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.color.r = 1;
    marker.color.g = 0.5;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.0);

    geometry_msgs::Point p1,p2;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0.05;
    p2.x = carrot.getX();
    p2.y = carrot.getZ();
    p2.z = 0.05;
    marker.points.push_back(p1);
    marker.points.push_back(p2);


    pub.publish(marker);
}

void CarrotPlanner::publishCmdVel(const geometry_msgs::Twist& cmd_vel, ros::Publisher& pub) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = TRACKING_FRAME;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cmd_vel";
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.type = visualization_msgs::Marker::ARROW;

    geometry_msgs::Point begin_vec, end_vec;
    begin_vec.x = 0;
    begin_vec.y = 0;
    begin_vec.z = 0.05;
    end_vec.x = 3*cmd_vel.linear.x;
    end_vec.y = 3*cmd_vel.linear.y;
    end_vec.z = 0.05 + 3*cmd_vel.linear.z;

    marker.points.push_back(begin_vec);
    marker.points.push_back(end_vec);

    pub.publish(marker);
}






