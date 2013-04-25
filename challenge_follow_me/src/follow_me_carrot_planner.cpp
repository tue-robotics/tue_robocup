#include "challenge_follow_me/follow_me_carrot_planner.h"

CarrotPlanner::CarrotPlanner(const std::string &name){
    initialize(name);
}

void CarrotPlanner::initialize(const std::string& name) {

    ros::NodeHandle private_nh("~/" + name);

    // parameters
    private_nh.param("max_vel_translation", MAX_VEL, 0.5);
    private_nh.param("max_acc_translation", MAX_ACC, 0.25);
    private_nh.param("max_vel_rotation", MAX_VEL_THETA, 0.5);
    private_nh.param("max_acc_rotation", MAX_ACC_THETA, 0.25);
    private_nh.param("xy_goal_tolerance", XY_GOALREGION, 0.1);
    private_nh.param("yaw_goal_tolerance", THETA_GOALREGION, 0.1);
    private_nh.param("still_max_vel", STILL_MAX_VEL, 0.01);

    private_nh.param("gain", GAIN, 0.9);

    STILL_MAX_VEL_SQ = STILL_MAX_VEL * STILL_MAX_VEL;

    carrot_pub_ = private_nh.advertise<visualization_msgs::Marker>("carrot", 1);
    cmd_vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    laser_scan_sub_ = private_nh.subscribe("/base_scan", 10, &CarrotPlanner::laserScanCallBack, this);

    TRACKING_FRAME = "/base_link";

    t_last_cmd_vel_ = ros::Time::now().toSec();

    initialized_ = true;
    laser_data_ready_ = false;

}

CarrotPlanner::~CarrotPlanner() {
}

bool CarrotPlanner::MoveToGoal(geometry_msgs::PoseStamped &goal){
    geometry_msgs::Twist cmd_vel;

    //! Set goal
    if (setGoal(goal)) {

        //! Compute velocity command and publish
        if(computeVelocityCommand(cmd_vel)) {
            /*if (cmd_vel.linear.x < 0 || cmd_vel.linear.x > MAX_VEL) {
                cmd_vel.linear.x = 0;
            }
            if (cmd_vel.linear.y < 0 || cmd_vel.linear.y > MAX_VEL) {
                cmd_vel.linear.y = 0;
            }*/
            ROS_INFO("Publishing velocity command: (x:%f, y:%f, th:%f)", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            cmd_vel_pub_.publish(cmd_vel);
            return true;
        }
    }

    return false;
}

bool CarrotPlanner::setGoal(geometry_msgs::PoseStamped &goal){

    //! Check frame of the goal
    if (goal.header.frame_id != TRACKING_FRAME){
        ROS_ERROR("Expecting goal in frame %s", TRACKING_FRAME.c_str());
        return false;
    }

    //! Determine pose of the goal
    goal_angle_ = tf::getYaw(goal.pose.orientation);
    goal_.setX(goal.pose.position.x);
    goal_.setY(goal.pose.position.y);
    goal_.setZ(goal.pose.position.z);

    ROS_DEBUG_STREAM("[setGoal] goal = " << goal_.getX() << " " << goal_.getY() << ", angle_ = " << goal_angle_);

    //! Publish marker
    publishCarrot(goal_, carrot_pub_);

    return true;

}

bool CarrotPlanner::computeVelocityCommand(geometry_msgs::Twist &cmd_vel){

    //! Must be initialized in constructor
    if(!initialized_){
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    // determine dt since last callback
    double time = ros::Time::now().toSec() + ros::Time::now().toNSec() / 1e9;
    double dt = 0;
    if (t_last_cmd_vel_ > 0) {
        dt = time - t_last_cmd_vel_;
    }
    t_last_cmd_vel_ = time;
    
    ROS_DEBUG_STREAM("goal before is clear line = " << goal_.getX() << " " << goal_.getY() << ", angle_ = " << goal_angle_);

    //! Check if the path is free
    if(!isClearLine(goal_)) {
        ROS_WARN("Path is not free");
        setZeroVelocity(cmd_vel);
        cmd_vel_pub_.publish(cmd_vel);
        return true;
    }

    ROS_INFO("Path is free");
    
    ROS_DEBUG_STREAM("goal before normalization = " << goal_.getX() << " " << goal_.getY() << ", angle_ = " << goal_angle_);

    //! Normalize position
    geometry_msgs::Twist goal_norm_msg;
    tf::Vector3 goal_norm;
    if (goal_.length() > 0.00001) {
        goal_norm = goal_.normalized();
        tf::vector3TFToMsg(goal_norm, goal_norm_msg.linear);
    } else {
        goal_norm = goal_;
    }
    ROS_INFO_STREAM("goal = " << goal_norm_msg.linear << ", angle_ = " << goal_angle_);

    //! Determine velocity
    determineDesiredVelocity(goal_, goal_angle_, last_cmd_vel_, dt, cmd_vel);
    last_cmd_vel_ = cmd_vel;

    return true;
}

bool CarrotPlanner::isClearLine(tf::Vector3 &goal){

    /// TODO: check laser data inbetween target and robot

    //! Check if laser data is avaibale
    if (!laser_data_ready_) {
        ROS_INFO("No laser data available: path considered blocked");
        return false;
    }

    //! Get number of beams and resolution LRF from most recent laser message
    int num_readings = laser_scan_.ranges.size();
    int num_incr = goal_angle_/laser_scan_.angle_increment; // Both in rad
    int index_beam_obst = num_readings/2 + num_incr;
    int width = 15;

    for (int i = index_beam_obst - width; i <= index_beam_obst + width; ++i){

        //! Check if the intended direction falls within the range of the LRF
        if (i < num_readings) {
            double dist_to_obstacle = laser_scan_.ranges[i];

            ROS_DEBUG("Distance at beam %d/%d is %f [m] (goal lies %f [m] ahead)", i, num_readings, dist_to_obstacle, goal_.length() - 0.1);

            if (dist_to_obstacle < goal_.length() - 0.1 && dist_to_obstacle > 0.15) {
                ROS_WARN("Obstacle detected at %f [m], whereas goal lies %f [m] ahead", dist_to_obstacle, goal_.length() - 0.1);
                return false;
            }
        }
    }

    //! Virtual wall in front of robot (0.5 [m])
    double d_wall = 0.5, r_robot = 0.35;
    int dth = atan2(r_robot, d_wall);
    int d_step = dth/laser_scan_.angle_increment;
    int beam_middle = num_readings/2;

    for (int j = beam_middle - d_step; j < beam_middle + d_step; j=j+4) {
        if (j < num_readings) {
            double dist_to_obstacle = laser_scan_.ranges[j];

            if (dist_to_obstacle < 0.5) {
                ROS_WARN("Object too close: %f [m]", dist_to_obstacle);
                return false;
            }
        }
    }

    return true;
}



void CarrotPlanner::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_scan){

    //! Only consider front laser (isn't this covered by selecting topic?)
    if(laser_scan->header.frame_id == "/front_laser"){
        laser_scan_ = *laser_scan;
        laser_data_ready_ = true;
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
    //double v_norm_sq = current_vel_trans.length2();

    // determine the maximum error of the orientation,
    // if this error is reached the robot stops and only controls the rotation

    double v_wanted_norm = 0;

    if (e_norm > 0) {
        v_wanted_norm = std::min(MAX_VEL, GAIN * sqrt(2 * e_norm * MAX_ACC));
        ROS_DEBUG("Updated v_wanted_norm to %f", v_wanted_norm);
    } else {
        ROS_DEBUG("e_norm = %f, e_theta = %f", e_norm, e_theta);
    }

    // make sure the wanted velocity has the direction towards the goal and the magnitude of v_wanted_norm
    tf::Vector3 vel_wanted;
    if (e_pos.length() > 0) {
        vel_wanted = e_pos.normalized();
    } else {
        vel_wanted.setX(0);
        vel_wanted.setY(0);
        vel_wanted.setZ(0);
    }
    vel_wanted *= v_wanted_norm;

    // check if the acceleration bound is violated
    tf::Vector3 vel_diff = vel_wanted - current_vel_trans;
    double acc_wanted = vel_diff.length() / dt;
    ROS_DEBUG("vel_diff = (%f,%f,%f), acc_wanted = %f", vel_diff.getX(), vel_diff.getY(), vel_diff.getZ(), acc_wanted);
    if (acc_wanted > MAX_ACC) {
        tf::vector3TFToMsg(current_vel_trans + vel_diff.normalized() * MAX_ACC * dt, cmd_vel.linear);
    } else {
        tf::vector3TFToMsg(vel_wanted, cmd_vel.linear);
    }

    // the rotation is always controlled
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = determineReference(e_theta, current_vel.angular.z, MAX_VEL_THETA, MAX_ACC_THETA, dt);

    ROS_INFO("Velocity command: (x:%f, y:%f, th:%f)", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
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
    ROS_INFO("Rotational velocity is %f", vel);

    return vel;
}

void CarrotPlanner::setZeroVelocity(geometry_msgs::Twist& cmd_vel) {
    //cmd_vel.angular.x = 0;
    //cmd_vel.angular.y = 0;
    //cmd_vel.angular.z = 0;

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
}

void CarrotPlanner::publishCarrot(const tf::Vector3& carrot, ros::Publisher& pub) {

    //! Create a marker message for the plan
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
    p2.y = carrot.getY();
    p2.z = 0.05;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    pub.publish(marker);

}






