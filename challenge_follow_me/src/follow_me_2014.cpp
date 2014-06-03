// Ros
#include <ros/ros.h>

// Find ROS pkgs
#include <ros/package.h>

// Messages
#include <text_to_speech/Speak.h>
#include "tue_pocketsphinx/Switch.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Bool.h"
#include "amigo_msgs/RGBLightCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Twist.h"

// Action client related
#include <actionlib/client/simple_action_client.h>
#include "robot_skill_server/ExecuteAction.h"
#include "amigo_head_ref/HeadRefAction.h"

// Services
#include "std_srvs/Empty.h"

// Follower
#include "challenge_follow_me/Follower.h"

// Problib conversions
#include "problib/conversions.h"

// STL
#include <vector>
#include <map>


namespace speech_state {
enum SpeechState {
    DRIVE,
    CONFIRM_LEAVE
};
}

struct RobotPose {
    double x;
    double y;
    double phi;

    RobotPose() {}

    RobotPose(double x_pos, double y_pos, double angle) :
        x(x_pos), y(y_pos), phi(angle) {}
};


// SETTINGS
const std::string ROBOT_BASE_FRAME = "/amigo/base_link";   // name of the robot's base link frame
const double T_WAIT_MAX_AFTER_LEAVE_CMD = 7.5;             // after an amigoleave command, resume following if no confirmation is heart after this time
const double T_MAX_NO_MOVE_BEFORE_TRYING_3D = 5.0;         // time robot stands still before move_base_3d is used instead of the carrot planner
const double MAX_ELEVATOR_WALL_DISTANCE_SIDE = 1.25;       // maximum distance of robot to left/right wall in elevator (used to detect elevator)
const double MAX_ELEVATOR_WALL_DISTANCE_FRONT = 2.5;       // maximum distance of robot to front wall in elevator (used to detect elevator)
const double ELEVATOR_INLIER_SUM = 2.5;                    // sum of the % laser points that should at least be within bounds for elevator to be detected (left/front/right)

// Speech
ros::ServiceClient srv_speech_;                                                   // Communication: Service that makes AMIGO speak
ros::ServiceClient speech_recognition_client_;                                    // Communication: Client for starting / stopping speech recognition
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak
ros::Subscriber sub_speech_;

// Actuation
actionlib::SimpleActionClient<robot_skill_server::ExecuteAction>* ac_skill_server_;
actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>* ac_head_ref_;
ros::Publisher cmd_vel_pub_;

// Visualization
ros::Publisher rgb_pub_;                                                          // Communicatino: Set color AMIGO
ros::Publisher location_marker_pub_;                                              // Communication: Marker publisher

// Follower
Follower* follower_;

// Tf
tf::TransformListener* listener_;												  // Tf listenter to obtain tf information to store locations

// Clear cost map
ros::ServiceClient srv_cost_map;

ros::Subscriber sub_laser_;

// Administration: speech
bool speech_recognition_turned_on_ = false;
speech_state::SpeechState speech_state_ = speech_state::DRIVE;
double t_last_speech_cmd_ = 0;

// Adminstration: other
std::string current_clr_;
bool left_elevator_ = false;
bool in_elevator_ = false;
double t_pause_ = 0;
bool emergency_button_pressed_ = true;
bool check_elevator_ = true; // only check every second time laser data is received


void rotateRobot(double desired_angle)
{
    // Fixed velocity
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0.5;

    // Convert angle from degree to rad
    desired_angle = desired_angle/180.0*3.1415;

    // Calculate rotating time
    double rotating_time = desired_angle/cmd_vel.angular.z;
    ROS_INFO("Rotate robot approximately %f degree (%f seconds)...", desired_angle, rotating_time);

    // Move
    double t_start_rotating = ros::Time::now().toSec();
    ros::Rate r(30);
    while (ros::Time::now().toSec() - t_start_rotating < rotating_time)
    {
        cmd_vel_pub_.publish(cmd_vel);
        r.sleep();
    }

    cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(cmd_vel);
    ROS_INFO("Done rotating");

}


bool moveHead(double pan, double tilt, bool block = true)
{

    //! Add head reference action
    double t_start = ros::Time::now().toSec();
    amigo_head_ref::HeadRefGoal head_ref;
    head_ref.goal_type = 1; // 1: pan tilt, 0 keep tracking
    head_ref.pan = pan;
    head_ref.tilt = tilt;
    ac_head_ref_->sendGoal(head_ref);
    if (block) ac_head_ref_->waitForResult(ros::Duration(3.0));

    if (ac_head_ref_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WARN("Head could not reach target position");
        ROS_INFO("moveHead(%f,%f,%s) took %f [ms]", pan, tilt, block?"true":"false", 1000*(ros::Time::now().toSec()-t_start));
        return false;
    }

    ROS_INFO("moveHead(%f,%f,%s) took %f [ms]", pan, tilt, block?"true":"false", 1000*(ros::Time::now().toSec()-t_start));
    return true;

}




std::string getSpeechStateName(speech_state::SpeechState ss)
{
    std::string name = "<unknown mode>";
    if (ss == speech_state::DRIVE) name = "DRIVE";
    else if (ss == speech_state::CONFIRM_LEAVE) name = "CONFIRM_LEAVE";

    return name;
}


bool transformPoseStamped(geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out, std::string frame_out)
{
    //! Transform goal to another frame
    try
    {
        listener_->transformPose(frame_out, in, out);
        ROS_INFO("Transformed (%f,%f) in %s to (%f,%f) in %s",
                 in.pose.position.x, in.pose.position.y, in.header.frame_id.c_str(),
                 out.pose.position.x, out.pose.position.y, out.header.frame_id.c_str());
    }
    catch (tf::TransformException& e)
    {
        ROS_WARN("Could not transform goal to map frame: %s", e.what());
        return false;
    }

    return true;
}



void setRGBLights(std::string color)
{

    ROS_DEBUG("AMIGO: %s", color.c_str());

    std_msgs::ColorRGBA clr_msg;

    if (color == "red") clr_msg.r = 255;
    else if (color == "green") clr_msg.g = 255;
    else if (color == "blue") clr_msg.b = 255;
    else if (color == "cyan")
    {
        clr_msg.r = 0;
        clr_msg.g = 255;
        clr_msg.b = 255;
    }
    else
    {
        ROS_INFO("Requested color \'%s\' for RGB lights unknown", color.c_str());
        return;
    }

    //! Send color command
    amigo_msgs::RGBLightCommand rgb_cmd;
    rgb_cmd.color = clr_msg;
    rgb_cmd.show_color.data = true;

    rgb_pub_.publish(rgb_cmd);

    //! Update global
    current_clr_ = color;

}



bool startSpeechRecognition()
{
    //! Determine path speech file
    std::string knowledge_path = ros::package::getPath("tue_knowledge");
    if (knowledge_path == "")
    {
        return false;
    }
    std::string restaurant_speech_path = knowledge_path + "/speech_recognition/follow_me/";

    //! Determine file name
    std::string file_name = "";
    if (speech_state_ == speech_state::DRIVE) file_name = "amigoleave";
    else if (speech_state_ == speech_state::CONFIRM_LEAVE) file_name = "yesnocontinue";
    else
    {
        ROS_WARN("Speecht state is unknown");
        return false;
    }

    //! Select appropriate speech file
    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::START;
    req.hidden_markov_model = "/usr/share/pocketsphinx/model/hmm/wsj1";
    req.dictionary = restaurant_speech_path + file_name + ".dic";
    req.language_model = restaurant_speech_path + file_name + ".lm";

    //! Toggle speech
    tue_pocketsphinx::Switch::Response resp;
    if (speech_recognition_client_.call(req, resp))
    {
        if (resp.error_msg == "")
        {
            ROS_INFO("Switched on speech recognition for %s", file_name.c_str());
            speech_recognition_turned_on_ = true;
        }
        else
        {
            ROS_WARN("Unable to turn on speech recognition: %s", resp.error_msg.c_str());
            return false;
        }
    }
    else
    {
        ROS_WARN("Service call for turning on speech recognition failed");
        return false;
    }

    //setRGBLights("green");

    return true;
}

bool stopSpeechRecognition()
{

    //! Turn off speech recognition
    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::STOP;
    tue_pocketsphinx::Switch::Response resp;
    if (speech_recognition_client_.call(req, resp))
    {
        if (resp.error_msg == "")
        {
            ROS_DEBUG("Switched off speech recognition");
            speech_recognition_turned_on_ = false;
        }
        else
        {
            ROS_WARN("Unable to turn off speech recognition: %s", resp.error_msg.c_str());
            return false;
        }
    }
    else
    {
        ROS_WARN("Unable to turn off speech recognition");
        return false;
    }

    //setRGBLights("red");

    return true;
}



/**
 * @brief amigoSpeak let AMIGO say a sentence
 * @param sentence
 */
void amigoSpeak(std::string sentence, bool block = true)
{

    std::string clr_back_up = current_clr_;
    setRGBLights("red");
    t_last_speech_cmd_ = ros::Time::now().toSec();

    ROS_INFO("AMIGO: \'%s\'", sentence.c_str());

    // If needed toggle recognition
    bool toggle_speech = speech_recognition_turned_on_;
    if (toggle_speech) stopSpeechRecognition();

    //! Call speech service (topic if srv fails)
    text_to_speech::Speak speak;
    speak.request.sentence = sentence;
    speak.request.language = "us";
    speak.request.character = "kyle";
    speak.request.voice = "default";
    speak.request.emotion = "normal";
    speak.request.blocking_call = block;
    if (!srv_speech_.call(speak))
    {
        std_msgs::String sentence_msgs;
        sentence_msgs.data = sentence;
        pub_speech_.publish(sentence_msgs);
    }

    if (toggle_speech) startSpeechRecognition();


    setRGBLights(clr_back_up);

}

void updateSpeechState(speech_state::SpeechState new_state)
{
    ROS_DEBUG("Update speech state from %s to %s", getSpeechStateName(speech_state_).c_str(), getSpeechStateName(new_state).c_str());
    speech_state_ = new_state;
    stopSpeechRecognition();
    startSpeechRecognition();
}



bool moveBase(double x, double y, double theta, double goal_radius = 0.1, double dt = 3.0)
{

    // Determine goal pose
    ROS_INFO("Received a move base goal: (%f,%f,%f)", x, y, theta);
    robot_skill_server::ExecuteGoal goal;
    std::stringstream cmd;
    cmd << "base.move(x=" << x << ",y=" << y << ", phi=" << theta << ", goal_area_radius=" << goal_radius << ")";
    goal.command = cmd.str();

    // Send goal
    double t_send_goal = ros::Time::now().toSec();
    ac_skill_server_->sendGoal(goal);
    ac_skill_server_->waitForResult(ros::Duration(dt));
    if(ac_skill_server_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WARN("Could not reach base pose within %f [s]", dt);
        ac_skill_server_->cancelAllGoals();
        moveHead(0, 0, true);
        return false;
    }

    ROS_INFO("Reached base goal after %f [s].", ros::Time::now().toSec()-t_send_goal);

    moveHead(0, 0, true);

    return true;

}



void moveToRelativePosition(double x, double y, double phi, double dt)
{
    geometry_msgs::PoseStamped goal_pos_in, goal_pos_map;
    goal_pos_in.header.frame_id = ROBOT_BASE_FRAME;
    goal_pos_in.header.stamp = ros::Time::now()-ros::Duration(0.5);
    goal_pos_in.pose.position.x = x;
    goal_pos_in.pose.position.y = y;
    tf::Quaternion q;
    q.setRPY(0, 0, phi);
    goal_pos_in.pose.orientation.x = q.getX();
    goal_pos_in.pose.orientation.y = q.getY();
    goal_pos_in.pose.orientation.z = q.getZ();
    goal_pos_in.pose.orientation.w = q.getW();
    transformPoseStamped(goal_pos_in, goal_pos_map, "/map");
    moveBase(goal_pos_map.pose.position.x, goal_pos_map.pose.position.y, tf::getYaw(goal_pos_map.pose.orientation), 0.5, dt);
}


void leaveElevator()
{

    double t_start = ros::Time::now().toSec();

    // First rotate (to avoid move base 3d problem: path found but robot does not move)
    rotateRobot(180);

    // Get position
    tf::StampedTransform location_start;
    try
    {
        // Look up transform
        listener_->lookupTransform("/map", ROBOT_BASE_FRAME, ros::Time(0), location_start);
        ROS_INFO("Current position is (%f,%f)", location_start.getOrigin().getX(), location_start.getOrigin().getY());
        
        

        // If the robot did barely move: try another set point
        double dx = 0, dy = 0;
        double y = 0;
        int fctr = 1;
        int count = 1;
        while (dx*dx+dy*dy < 1.0 && count < 5)
        {
            if (count == 1) ROS_INFO("Trying to move out of the elevator: attempt %d", count);
            else ROS_WARN("Trying to move out of the elevator: attempt %d", count);

            // Try to move to this position
            //moveToRelativePosition(-3.5, fctr*y, 3.14, 35.0);
            moveToRelativePosition(2.5, fctr*y, 3.14, 30.0);

            // Get current robot position
            try
            {
                tf::StampedTransform location_now;
                listener_->lookupTransform("/map", ROBOT_BASE_FRAME, ros::Time(0), location_now);
                dx = std::fabs(location_now.getOrigin().getX() - location_start.getOrigin().getX());
                dy = std::fabs(location_now.getOrigin().getY() - location_start.getOrigin().getY());
            }
            catch (tf::TransformException& e) {ROS_WARN("While driving out of the elevator: %s", e.what());}

            // Update goal if the robot didn't move
            y += 0.25;
            fctr *= -1.0;
            ++count;
        }

        if (count == 5) ROS_ERROR("I cannot leave the elevator the way I want it, I will continue from here");

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("No tranform /map - /amigo/base_link: %s", ex.what());
        moveToRelativePosition(-2.5, 0.0, 3.14, 35.0);
        return;
    }

    ROS_INFO("Leaving the elevator took %f [s]", ros::Time::now().toSec()-t_start);


}




void driveAroundCrowd()
{

    double t_start = ros::Time::now().toSec();

    // Do a random rotation (to avoid move base 3d problem: path found but robot does not move)
    rotateRobot(90);

    // Get position
    tf::StampedTransform location_start;
    try
    {
        // Look up transform
        listener_->lookupTransform("/map", ROBOT_BASE_FRAME, ros::Time(0), location_start);
        ROS_INFO("Current position is (%f,%f)", location_start.getOrigin().getX(), location_start.getOrigin().getY());
        
        

        // If the robot did barely move: try another set point
        double dx = 0, dy = 0;
        double y = 0;
        int fctr = 1;
        int count = 1;
        while (dx*dx+dy*dy < 1.0 && count < 5)
        {
            if (count == 1) ROS_INFO("Trying to move around the crowd: attempt %d", count);
            else ROS_WARN("Trying to move around the crowd: attempt %d", count);

            // Try to move to this position
            moveToRelativePosition(2.5, fctr*y, -1.57, 45.0);

            // Get current robot position
            try
            {
                tf::StampedTransform location_now;
                listener_->lookupTransform("/map", ROBOT_BASE_FRAME, ros::Time(0), location_now);
                dx = std::fabs(location_now.getOrigin().getX() - location_start.getOrigin().getX());
                dy = std::fabs(location_now.getOrigin().getY() - location_start.getOrigin().getY());
            }
            catch (tf::TransformException& e) {ROS_WARN("While driving around the crowd: %s", e.what());}

            // Update goal if the robot didn't move
            y += 0.25;
            fctr *= -1.0;
            ++count;
        }

        if (count == 5) ROS_ERROR("I cannot drive aroud the crowd, I will continue from here");

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("No tranform /map - /amigo/base_link: %s", ex.what());
        moveToRelativePosition(2.5, 0.0, 0.0, 35.0);
        return;
    }

    ROS_INFO("Driving around the crowd took %f [s]", ros::Time::now().toSec()-t_start);


}



void speechCallback(std_msgs::String res)
{

    t_last_speech_cmd_ = ros::Time::now().toSec();

    //! Only consider non-empty answers
    std::string answer = res.data;
    ROS_INFO("Received command: %s", answer.c_str());
    if (answer.empty()) return;

    // ROBOT GOT A COMMAND TO STOP AND COULD BE IN AN ELEVATOR
    if (speech_state_ == speech_state::DRIVE && answer == "amigoleave")
    {
        if (in_elevator_)
        {
            // stop robot
            follower_->pause();
            ROS_INFO("Paused follower!");

            // Get location and side
            updateSpeechState(speech_state::CONFIRM_LEAVE);
            amigoSpeak("Should I leave the elevator?");
            setRGBLights("green");

            t_pause_ = ros::Time::now().toSec();
        }
        else ROS_INFO("Heard stop command but in_elevator is false");
    }

    // ASKED FOR CONFIRMATION ON WHETHER OR NOT TO LEAVE THE ELEVATOR
    else if (speech_state_ == speech_state::CONFIRM_LEAVE)
    {
        // Get first word from the answer
        size_t position = answer.find_first_of(" ");
        if (position > 0 && position <= answer.size()) answer = std::string(answer.c_str(), position);

        // MUST LEAVE THE ELEVATOR
        if (answer == "yes")
        {
            follower_->pause();
            amigoSpeak("I will leave the elevator. Please wait until I call you", false);

            //! Leave the elevator
            leaveElevator();

            //! Shutdown the speech
            sub_speech_.shutdown();
            left_elevator_ = true;
            in_elevator_ = false;

            //! Done with the elevator
            amigoSpeak("You can leave the elevator", true);
            follower_->reset(1.0);
            sub_laser_.shutdown();


        }
        // MISUNDERSTOOD
        else
        {
            // Update the state
            updateSpeechState(speech_state::DRIVE);

            // Robot should follow
            follower_->resume();

            setRGBLights("green");
        }

    }
    else
    {
        ROS_WARN("In speech callback but state is not defined!");
    }

}


bool resetSpindlePosition()
{
    double std_spindle_pos = 0.35;

    // Determine goal pose
    ROS_INFO("Reset spindle position!");
    robot_skill_server::ExecuteGoal goal;
    std::stringstream cmd;
    cmd << "move_spindle(" << std_spindle_pos << ")";
    goal.command = cmd.str();

    // Send goal
    ac_skill_server_->sendGoal(goal);
    //ac_skill_server_->waitForResult(ros::Duration(4.0));
    //if(ac_skill_server_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    //{
    //    ROS_WARN("Could not spindle position %f within 4 [s]", std_spindle_pos);
    //    return false;
    //}

    return true;
}






void restartSpeechIfNeeded()
{
    if (ros::Time::now().toSec() - t_last_speech_cmd_ > 10.0 && !left_elevator_)
    {
        // Restart speech recognition
        stopSpeechRecognition();
        startSpeechRecognition();
        t_last_speech_cmd_ = ros::Time::now().toSec();

        // Inform user
        std::string clr = current_clr_;
        setRGBLights("blue");
        ros::Duration(0.5).sleep();
        setRGBLights(clr);
        ROS_INFO("Restarted speech for state %s!", getSpeechStateName(speech_state_).c_str());
    }
}




void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg)
{

    if (!check_elevator_)
    {
        check_elevator_ = true;
        return;
    }
    
    //ROS_INFO("Laser contains %zu beams, angle [%f,%f]", 
	//	laser_scan_msg->ranges.size(), laser_scan_msg->angle_min, 
	//	laser_scan_msg->angle_min+laser_scan_msg->ranges.size()*laser_scan_msg->angle_increment);


    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    //   ELEVATOR DETECTOR
    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    const double PI = 3.14;
    std::vector<int> num_total_points(3, 0);
    std::vector<int> num_points_in_bounds(3, 0);

    double angle = laser_scan_msg->angle_min;
    for(int i = 0; i < (int)laser_scan_msg->ranges.size(); ++i)
    {
		//double previous_range = laser_scan_msg->ranges[std::max(i-1,0)];
        double range = laser_scan_msg->ranges[i];

        // check left of robot
        if (angle > -0.4 * PI && angle < -0.15 * PI)
        {
            num_total_points[0]++;
            if (range < MAX_ELEVATOR_WALL_DISTANCE_SIDE) num_points_in_bounds[0]++;
        }
        // check in front of robot
        else if (angle > -0.15 * PI && angle < 0.15 * PI)
        {
            num_total_points[1]++;
            if (range < MAX_ELEVATOR_WALL_DISTANCE_FRONT) num_points_in_bounds[1]++;

        }
        // check right of robot
        else if (angle > 0.15 * PI && angle < 0.4 * PI)
        {
            num_total_points[2]++;
            if (range < MAX_ELEVATOR_WALL_DISTANCE_SIDE) num_points_in_bounds[2]++;
        }

        angle += laser_scan_msg->angle_increment;
    }

    // check if all parts have enough 'inliers'
    double sum_ratios = 0;
    for (unsigned int i = 0; i < num_total_points.size(); ++i)
    {
		ROS_DEBUG("\tratio is %f (based on %d points)", (double)num_points_in_bounds[i] / num_total_points[i], num_total_points[i]);
        sum_ratios += (double)num_points_in_bounds[i] / num_total_points[i];
    }
    ROS_DEBUG("\tin_elevator_ is %s", in_elevator_?"true":"false");

    // Inside elevator: cyan, otherwise green
    in_elevator_ = (sum_ratios>ELEVATOR_INLIER_SUM);
    if (in_elevator_) setRGBLights("cyan");
	else  setRGBLights("green");
	check_elevator_ = false;
}



void emergencyCallback(const std_msgs::Bool::ConstPtr& em_button_msg)
{
    // If the emergency button is released, boolean is false
    if (!em_button_msg->data)
    {
        emergency_button_pressed_ = false;
        return;
    }

    ROS_DEBUG("Emergency button is pressed...");
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "follow_me_2014");
    ros::NodeHandle nh;

    //! RGB lights
    rgb_pub_ = nh.advertise<amigo_msgs::RGBLightCommand>("/user_set_rgb_lights", 1);
    setRGBLights("blue");

    //! Transforms
    listener_ = new tf::TransformListener();

    //! Topic/srv that make AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/text_to_speech/input", 10);
    srv_speech_ =  nh.serviceClient<text_to_speech::Speak>("/text_to_speech/speak");
    ROS_INFO("Publisher/service client for text to speech started");

    //! Start speech recognition
    speech_recognition_client_ = nh.serviceClient<tue_pocketsphinx::Switch>("/pocketsphinx/switch");
    speech_recognition_client_.waitForExistence();
    sub_speech_ = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 1, speechCallback);

    //! Skill server action client
    ROS_INFO("Connecting to the skill server...");
    ac_skill_server_ = new actionlib::SimpleActionClient<robot_skill_server::ExecuteAction>("/amigo/execute_command", true);
    ac_skill_server_->waitForServer();
    ROS_INFO("Connected!");

    //! Head
    ROS_INFO("Connecting to head ref action server...");
    ac_head_ref_ = new actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>("head_ref_action", true);
    ac_head_ref_->waitForServer();
    ROS_INFO("Connected!");
    moveHead(0, 0, true);

    //! Allow for rotations
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/amigo/base/references", 1);

    //! Laser data
    sub_laser_ = nh.subscribe<sensor_msgs::LaserScan>("/amigo/base_front_laser", 10, laserCallback);

	//! Create follower (which connects with WIRE)
	//! Construct the follower
    ROS_INFO("Constructing the follower...");
    follower_ = new Follower(nh, ROBOT_BASE_FRAME, false);
    ROS_INFO("done!");
    
    //! Wait for the emergency switch to be released
    ros::Subscriber sub_emergency = nh.subscribe<std_msgs::Bool>("/amigo/emergency_switch", 10, emergencyCallback);
    ros::Rate loop_rate_slow(5);
    ROS_INFO("Waiting for emegency button to be released...");
    while (emergency_button_pressed_)
    {
        ros::spinOnce();
        loop_rate_slow.sleep();
    }
    sub_emergency.shutdown();
    ROS_INFO("Emergency button released!");
    
    //! Clear cost map interface
    srv_cost_map = nh.serviceClient<std_srvs::Empty>("/move_base_3d/reset");
    srv_cost_map.waitForExistence(ros::Duration(2.0));
    std_srvs::Empty empty_srv;
    if (!srv_cost_map.exists() && !srv_cost_map.call(empty_srv)) ROS_WARN("Cannot clear the cost map");
    
    //! Reset spindle
    resetSpindlePosition();

    //! Clear the world model
    ros::ServiceClient reset_wire_client = nh.serviceClient<std_srvs::Empty>("/wire/reset");
    std_srvs::Empty srv;
    if (!reset_wire_client.call(srv)) ROS_WARN("Failed to clear world model");

    //! Start follower
    if (!follower_->start())
    {
        ROS_ERROR("Could not start the follower!");
        return -1;
    }

    //! Wait for operator
    bool operator_found = false;
    while (!operator_found)
    {
        operator_found = follower_->update();
        loop_rate_slow.sleep();
        setRGBLights("green");

    }

    stopSpeechRecognition();
    startSpeechRecognition();
    setRGBLights("green");
    t_last_speech_cmd_ = ros::Time::now().toSec();
    ROS_INFO("Started speech recognition");

    //! Start Following
    unsigned int n_move_base_3d_tries = 0;
    bool drive = false;
    double t_start_no_move = 0;
    ros::Rate loop_rate_fast(25);
    while (ros::ok())
    {
        //! Get information from topics
        ros::spinOnce();


        //! To avoid a deadlock after a false speech command
        if (speech_state_ == speech_state::CONFIRM_LEAVE &&
                ros::Time::now().toSec() - t_pause_ > T_WAIT_MAX_AFTER_LEAVE_CMD)
        {
            ROS_WARN("I assume I misunderstood, I will continue following");
            speech_state_ = speech_state::DRIVE;
            follower_->resume();
        }


        //! Update the follower
        bool non_zero_vel = follower_->update();


        //! If the robot did not move for a while
        if (!non_zero_vel)
        {
            if (drive) t_start_no_move = ros::Time::now().toSec();
            ROS_DEBUG("Robot does not move");
            drive = false;

            // If the robot did not move for a while
            if (ros::Time::now().toSec() - t_start_no_move > T_MAX_NO_MOVE_BEFORE_TRYING_3D)
            {
                ROS_INFO("left_elevator_ : %i!" , left_elevator_);
                // Inside the elevator: do not use 3d navigation here!
                if (in_elevator_)
                {
                    ROS_INFO("In the elevator. TODO: slowly move forward.");
                }
                // Outside the elevator, see if and how move base 3d must be used
                else
                {

                    // AFTER THE ELEVATOR: move forward (around the crowd)
                    if (left_elevator_)
                    {
                        ROS_INFO("I think I am at the crowd, I will try to drive around the crowd");
                        amigoSpeak("I think I am at the crowd, I will try to drive around the crowd");
                        follower_->pause();
                        driveAroundCrowd();
                        follower_->reset(1.0);
                        ROS_INFO("Done with the 3d nav move around the crowd");

                    }
                    // BEFORE THE ELEVATOR: just try to move (there is something in the way)
                    else
                    {
                        // Get position operator
                        ROS_WARN("Robot did not move for %f [s], trying move_base_3d to plan around obstacle", ros::Time::now().toSec() - t_start_no_move);
                        double x = 0, y = 0, phi = 0;
                        follower_->getCurrentOperatorPosition(x, y, phi, ROBOT_BASE_FRAME);
                        ROS_INFO("Operator at (x,y,phi) = (%f,%f,%f)", x, y, phi);

                        // Go to the position:
                        moveToRelativePosition(0.65*x, 0.65*y, phi, 7.5);
                        ROS_INFO("Done with the 3d nav move");
                    }

                    // There is might be a problem with move base 3d: robot can not get to operator
                    if (n_move_base_3d_tries > 4) ROS_WARN("Follow me: there might be a move base 3d problem!");
                }
            }
            // Else: robot did not move for a while but that is not a problem

        }
        // Else: robot is moving, do adminstration
        else
        {
            drive = true;
            n_move_base_3d_tries = 0;
        }

        //! To ensure speech keeps working restart it every now and then
        restartSpeechIfNeeded();

        //! Wait
        loop_rate_fast.sleep();
    }

    delete ac_skill_server_;
    delete ac_head_ref_;

    return 0;
}

