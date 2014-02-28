#include "wire_interface/Client.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>

// Action client
#include <actionlib/client/simple_action_client.h>

// Actions
#include <tue_move_base_msgs/MoveBaseAction.h>
#include <amigo_head_ref/HeadRefAction.h>
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "amigo_actions/AmigoGripperCommandAction.h"

// Messages
#include "amigo_msgs/arm_joints.h"
#include "amigo_msgs/RGBLightCommand.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ColorRGBA.h>
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <text_to_speech/Speak.h>
#include "tue_pocketsphinx/Switch.h"

// Services
#include <std_srvs/Empty.h>
#include "perception_srvs/StartPerception.h"
#include "speech_interpreter/GetInfo.h"
#include "challenge_restaurant/SmachStates.h"

// Problib conversions
#include "problib/conversions.h"

// Follower
#include "challenge_follow_me/Follower.h"

using namespace std;

#include <iostream>
#include <vector>
#include <cstring>
#include <fstream>

//! Settings
unsigned int N_ORDERS = 3;                      // Number of orders robot needs to take
const int TIME_OUT_GUIDE_LOST = 3.5;            // Time interval without updates after which operator is considered to be lost
const double DISTANCE_GUIDE = 0.7;              // Distance AMIGO keeps towards guide
const double WAIT_TIME_GUIDE_MAX = 15.0;        // Maximum waiting time for guide to return
const string NAVIGATION_FRAME = "/base_link";   // Frame in which navigation goals are given IF NOT BASE LINK, UPDATE PATH IN moveTowardsPosition()
const double FOLLOW_RATE = 20;                  // Rate at which the move base goal is updated
double FIND_RATE = 5;                           // Rate check for guide at start of the challenge
const double TYPICAL_GUIDE_X = 1.0;             // Expected x-position operator, needed when looking for operator
const double TYPICAL_GUIDE_Y = 0;               // Expected y-position operator, needed when looking for operator


//! Globals
bool freeze_amigo_ = false;                                                       // Bookkeeping: true if stop command is confirmed
bool candidate_freeze_amigo_ = false;                                             // Bookkeeping: true if robot is asked to stop
bool stored_location = false;                                                     // Bookkeeping: true if robot another location must be learned
bool finished = false;                                                            // Bookkeeping: true if robot must go to ordering location
bool candidate_ask_side = false;                                                  // Bookkeeping: true if robot will ask for a side
bool ask_side = false;                                                            // Bookkeeping: true if robot wants confirmation for a side
int state = 0;                                                                    // Bookkeeping: state
unsigned int n_locations_deliver_ = 0;                                            // Bookkeeping: number of locations learned
unsigned int n_shelves_ = 0;                                                      // Bookkeeping: number of shelves learned
double t_freeze_ = 0;                                                             // Bookkeeping: time the robot is frozen
bool speech_recognition_on = false;                                               // Bookkeeping: speech switched on/off
string current_side_ = "";                                                        // Bookkeeping: current side
unsigned int state_speech_ = 0;
unsigned int n_fails_side = 0;                                                    // Bookkeeping: avoid infinite 'ask side' loops
string current_location_name_;                                                    // Bookkeeping: current location name

//! Action clients
actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* move_base_ac_;           // Communication: Move base action client
actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>* head_ref_ac_;                 // Communication: Head ref action client
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* left_arm_ac_;     // Communication: Left arm action client
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* right_arm_ac_;    // Communication: Right arm action client
actionlib::SimpleActionClient<amigo_actions::AmigoGripperCommandAction>* gripper_left_ac_;  // Communication: left gripper action client
actionlib::SimpleActionClient<amigo_actions::AmigoGripperCommandAction>* gripper_right_ac_; // Communication: Right gripper action client

//! Service clients
ros::ServiceClient pein_client_;                                                  // Communication: Toggle perception
ros::ServiceClient srv_speech_;                                                   // Communication: Service that makes AMIGO speak
ros::ServiceClient reset_wire_client_;                                            // Communication: Client that enables reseting WIRE
ros::ServiceClient speech_recognition_client_;                                    // Communication: Client for starting / stopping speech recognition
ros::ServiceClient speech_client_;                                                // Communication: Communication with the speech interpreter
ros::ServiceClient grab_machine_client_;                                          // Communication: Connection with (python) grab machine

//! Publishers
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak
ros::Publisher head_ref_pub_;                                                     // Communication: Look to intended driving direction
ros::Publisher location_marker_pub_;                                              // Communication: Marker publisher
ros::Publisher rgb_pub_;                                                          // Communication: Publisher rgb lights

//! Tf
tf::TransformListener* listener;												  // Tf listenter to obtain tf information to store locations

psi::Client* reasoner_client;

//! Maps storing orders and locations
map<string, tf::StampedTransform> location_map_;
map<int, pair<string, string> > order_map_;

//! Function prototypes to avoid errors
void moveTowardsPosition(pbl::PDF& pos, double offset);
bool moveHead(double pan, double tilt);
bool callInterpreter(string type, string& answer);


void setRGBLights(string color)
{

    std_msgs::ColorRGBA clr_msg;

    if (color == "red") {
        clr_msg.r = 255;
    } else if (color == "green") {
        clr_msg.g = 255;
    } else if (color == "blue") {
        clr_msg.g = 255;
    } else if (color == "yellow") {
        clr_msg.r = 255;
        clr_msg.g = 255;
    }else {
        ROS_INFO("Requested color \'%s\' for RGB lights unknown", color.c_str());
        return;
    }

    //! Send color command
    amigo_msgs::RGBLightCommand rgb_cmd;
    rgb_cmd.color = clr_msg;
    rgb_cmd.show_color.data = true;

    rgb_pub_.publish(rgb_cmd);

}




void resetRGBLights()
{
    //! Send color command
    amigo_msgs::RGBLightCommand rgb_cmd;
    rgb_cmd.show_color.data = false;
    rgb_pub_.publish(rgb_cmd);
}

trajectory_msgs::JointTrajectoryPoint setArmReference(double r0, double r1, double r2, double r3, double r4, double r5, double r6)
{

    trajectory_msgs::JointTrajectoryPoint arm_msg;
    arm_msg.positions.push_back(r0);
    arm_msg.positions.push_back(r1);
    arm_msg.positions.push_back(r2);
    arm_msg.positions.push_back(r3);
    arm_msg.positions.push_back(r4);
    arm_msg.positions.push_back(r5);
    //arm_msg.positions.push_back(r6);

    return arm_msg;
}


bool moveArm(string arm, string pose) {


    control_msgs::FollowJointTrajectoryGoal arm_ref;

    // Check input pose
    if (pose == "drive") {
        arm_ref.trajectory.points.push_back(setArmReference(-0.1, -0.2, 0.2, 0.8, 0.0, 0.0, 0.0));
    } else if (pose == "carry") {
        arm_ref.trajectory.points.push_back(setArmReference(-0.4, -0.38, 0.51, 1.56, -0.2, 0.52, -0.38));
        //} else if (pose == "give") {
        //    moveArm(arm, "carry");
        //    arm_ref.trajectory.points.push_back(setArmReference(-0.93 , 1.06 , 0.07 , 0.91 , -1.02 , -0.19 , -0.36));
    } else {
        ROS_INFO("Arm pose for %s arm unknown: \'%s\'", arm.c_str(), pose.c_str());
        ROS_INFO("return false 1");
        return false;
    }

    // Send goal(s)
    if (arm == "left") {
        left_arm_ac_->sendGoal(arm_ref);
        left_arm_ac_->waitForResult(ros::Duration(20.0));
    }
    else if (arm == "right") {
        right_arm_ac_->sendGoal(arm_ref);
        right_arm_ac_->waitForResult(ros::Duration(20.0));
    }
    else if (arm == "both") {
        left_arm_ac_->sendGoal(arm_ref);
        right_arm_ac_->sendGoal(arm_ref);
        left_arm_ac_->waitForResult(ros::Duration(20.0));
        right_arm_ac_->waitForResult(ros::Duration(20.0));
    }


    // Get feedback

    // Left arm
    if(arm == "left" && left_arm_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("Left arm could not reach target position");
        ROS_INFO("return false 2");
        return false;
    }
    // Right arm
    else if(arm == "right" && left_arm_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("Left arm could not reach target position");
        ROS_INFO("return false 3");
        return false;
    }
    // Both arms
    else if(arm == "both") {
        bool both_ok = true;

        // Left
        if (left_arm_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_WARN("Left arm could not reach target position");
            ROS_INFO("return false 4");
            both_ok = false;
        }

        // Right
        if (right_arm_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_WARN("Right arm could not reach target position");
            ROS_INFO("return false 5");
            both_ok = false;
        }

        return both_ok;

    }

    return true;

}

bool moveGripper(string side, string direction) {

    //! Set action
    amigo_actions::AmigoGripperCommandGoal gripper_ref;
    if (direction == "open") {
        gripper_ref.command.direction = -1;
    }
    else if (direction == "close") {
        gripper_ref.command.direction = 1;
    }
    else {
        ROS_WARN("Received unknown gripper command \'%s\'", direction.c_str());
        return false;
    }

    //! Send goal
    gripper_ref.command.max_torque = 50.0;
    if (side == "left") {
        gripper_left_ac_->sendGoal(gripper_ref);
        gripper_left_ac_->waitForResult(ros::Duration(5.0));
    } else if (side == "right") {
        gripper_right_ac_->sendGoal(gripper_ref);
        gripper_right_ac_->waitForResult(ros::Duration(5.0));
    } else {
        ROS_WARN("Gripper side \'%s\' unknown, must be either left or right", side.c_str());
    }

    if ((side == "left" && gripper_left_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) ||
            (side == "right" && gripper_right_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)) {
        ROS_WARN("Gripper (%s) could not reach target position", side.c_str());
        return false;
    }

    return true;


}

bool startSpeechRecognition() {
    std::string knowledge_path = ros::package::getPath("tue_knowledge");
    if (knowledge_path == "") {
        return false;
    }

    if (state_speech_ == 0) {
        // amigostop (to shelf/delivery location)
    } else if (state_speech_ == 1) {
        // yes/no (amigostop)
    } else if (state_speech_ == 2) {
        // yes/no (is this a shelf)
    } else if (state_speech_ == 3) {
        // left/right/front (which side?)
    } else if (state_speech_ == 4) {
        // yes/no (is left/right/front correct?)
    } else if (state_speech_ == 5) {
        // yes/no (another location?)
    } else if (state_speech_ == 6) {
        // amigostop (but to ordering location)
    } else if (state_speech_ == 7) {
        // yes/no (amigostop at ordering location)
    }

    std::string restaurant_speech_path = knowledge_path + "/speech_recognition/restaurant/";

    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::START;
    req.hidden_markov_model = "/usr/share/pocketsphinx/model/hmm/wsj1";
    req.dictionary = restaurant_speech_path + "restaurant.dic";
    req.language_model = restaurant_speech_path + "restaurant.lm";

    tue_pocketsphinx::Switch::Response resp;
    if (speech_recognition_client_.call(req, resp)) {
        if (resp.error_msg == "") {
            ROS_INFO("Switched on speech recognition");
            speech_recognition_on = true;
        } else {
            ROS_WARN("Unable to turn on speech recognition: %s", resp.error_msg.c_str());
            return false;
        }
    } else {
        ROS_WARN("Service call for turning on speech recognition failed");
        return false;
    }

    return true;
}

bool stopSpeechRecognition() {
    // Turn off speech recognition
    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::STOP;

    tue_pocketsphinx::Switch::Response resp;
    if (speech_recognition_client_.call(req, resp)) {
        if (resp.error_msg == "") {
            ROS_INFO("Switched off speech recognition");
            speech_recognition_on = false;
        } else {
            ROS_WARN("Unable to turn off speech recognition: %s", resp.error_msg.c_str());
            return false;
        }
    } else {
        ROS_WARN("Unable to turn off speech recognition");
        return false;
    }

    return true;
}



/**
 * @brief amigoSpeak let AMIGO say a sentence
 * @param sentence
 */
void amigoSpeak(string sentence, bool block = true) {

    ROS_INFO("AMIGO: \'%s\'", sentence.c_str());

    setRGBLights("red");

    bool toggle_speech = speech_recognition_on;
    if (toggle_speech) stopSpeechRecognition();

    //! Call speech service
    text_to_speech::Speak speak;
    speak.request.sentence = sentence;
    speak.request.language = "us";
    speak.request.character = "kyle";
    speak.request.voice = "default";
    speak.request.emotion = "normal";
    speak.request.blocking_call = block;

    if (!srv_speech_.call(speak)) {
        std_msgs::String sentence_msgs;
        sentence_msgs.data = sentence;
        pub_speech_.publish(sentence_msgs);
    }

    if (toggle_speech) startSpeechRecognition();


    setRGBLights("green");

}




void createMarkerWithLabel(string label, tf::StampedTransform& pose, double r, double g, double b, visualization_msgs::MarkerArray& array) {

    // Geometric marker
    visualization_msgs::Marker marker;
    marker.ns = "restaurant/location_markers";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.header.frame_id = "/map";
    marker.id = location_map_.size();
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1;
    marker.pose.position.x = pose.getOrigin().x();
    marker.pose.position.y = pose.getOrigin().y();
    marker.pose.orientation.w = pose.getRotation().getW();
    marker.pose.orientation.x = pose.getRotation().getX();
    marker.pose.orientation.y = pose.getRotation().getY();
    marker.pose.orientation.z = pose.getRotation().getZ();
    array.markers.push_back(marker);

    // Text label
    visualization_msgs::Marker marker_txt = marker;
    marker_txt.scale.z = 0.1;
    marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_txt.text = label;
    marker_txt.id = location_map_.size()*10;
    marker_txt.pose.position.z *= 1.5;
    array.markers.push_back(marker_txt);

    ROS_INFO("Added marker with color (%f,%f,%f), size (%f,%f,%f) and label %s",
             marker.color.r, marker.color.g, marker.color.b, marker.scale.x, marker.scale.y, marker.scale.z, marker_txt.text.c_str());

}







bool moveTowardsPositionMap(tf::StampedTransform pos) {


    geometry_msgs::PoseStamped base_pose;
    base_pose.pose.position.x = pos.getOrigin().getX();
    base_pose.pose.position.y = pos.getOrigin().getY();
    base_pose.header.frame_id = "/map";

    //! Transform quaternion to message
    tf::Quaternion q1 = pos.getRotation();
    tf::quaternionTFToMsg(q1, base_pose.pose.orientation);

    //! Add to base goal
    tue_move_base_msgs::MoveBaseGoal base_goal;
    base_goal.path.push_back(base_pose);
    move_base_ac_->sendGoal(base_goal);
    move_base_ac_->waitForResult(ros::Duration(60.0));

    if(move_base_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("Could not reach target position");
        return false;
    }

    return true;

}



bool togglePein(vector<string> modules) {

    perception_srvs::StartPerception pein_srv;

    if (modules.empty())
    {
        pein_srv.request.modules.push_back("");
    }
    else {
        vector<string>::const_iterator it = modules.begin();
        for (; it != modules.end(); ++it)
        {
            pein_srv.request.modules.push_back(*it);
        }
    }

    if (pein_client_.call(pein_srv))
    {
        ROS_INFO("Switched on pein_modules:");
        vector<string>::const_iterator it = modules.begin();
        for (; it != modules.end(); ++it)
        {
            ROS_INFO("\t%s", it->c_str());
        }

        return true;

    }
    else
    {
        ROS_WARN("Could not switch on pein_modules:");
        vector<string>::const_iterator it = modules.begin();
        for (; it != modules.end(); ++it)
        {
            ROS_WARN("\t%s", it->c_str());
        }
    }

    return false;

}



void amigoRotate(double delta)
{

    ROS_INFO("Rotate for %f [s]...", delta);
    double t_start = ros::Time::now().toSec();
    double freq = 20;
    ros::Duration pause(1.0/freq);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0.4;

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/amigo/base/references", 1);

    while (ros::Time::now().toSec()-t_start < delta)
    {
        cmd_vel_pub.publish(cmd_vel);
        pause.sleep();
    }

    ROS_INFO("Done rotating");
}



bool moveHead(double pan, double tilt) {

    //! Add head reference action
    amigo_head_ref::HeadRefGoal head_ref;
    head_ref.goal_type = 1; // 1: pan tilt, 0 keep tracking
    head_ref.pan = pan;
    head_ref.tilt = tilt;
    head_ref_ac_->sendGoal(head_ref);
    head_ref_ac_->waitForResult(ros::Duration(2.0));

    if(head_ref_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("Head could not reach target position");
        return false;
    }

    return true;

}


void speechCallback(std_msgs::String res)
{

    ROS_INFO("Received command: %s", res.data.c_str());
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "restaurant_generic");
    ros::NodeHandle nh;

    ROS_INFO("Started Restaurant");

    //! Location markers
    location_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/restaurant/location_markers", 10);

    /// Tf listener
    listener = new tf::TransformListener();

    //! Connect to interpreter
    speech_client_ = nh.serviceClient<speech_interpreter::GetInfo>("interpreter/get_info_user");

    //! RGB lights
    rgb_pub_ = nh.advertise<amigo_msgs::RGBLightCommand>("/user_set_rgb_lights", 1);

    //! Topic/srv that make AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/text_to_speech/input", 10);
    srv_speech_ =  nh.serviceClient<text_to_speech::Speak>("/text_to_speech/speak");
    ROS_INFO("Publisher/service client for text to speech started");

    //! Head ref action client
    ROS_INFO("Connecting to head ref action server...");
    head_ref_ac_ = new actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>("head_ref_action", true);
    head_ref_ac_->waitForServer();
    ROS_INFO("Connected!");

    //! Arm action clients
    ROS_INFO("Connecting to joint trajectory action servers...");
    left_arm_ac_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("joint_trajectory_action_left", true);
    right_arm_ac_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("joint_trajectory_action_right", true);
    left_arm_ac_->waitForServer();
    right_arm_ac_->waitForServer();
    ROS_INFO("Connected!");

    /// Reset arms

    if (!moveArm("both", "drive")) {
        amigoSpeak("I am not able to move my arms to the drive position");
    } else {
        ROS_INFO("Arms in driving position");
    }

    //! Gripper action client
    ROS_INFO("Connecting to gripper action servers...");
    gripper_left_ac_ = new actionlib::SimpleActionClient<amigo_actions::AmigoGripperCommandAction>("gripper_server_left", true);
    gripper_right_ac_ = new actionlib::SimpleActionClient<amigo_actions::AmigoGripperCommandAction>("gripper_server_right", true);
    gripper_left_ac_->waitForServer();
    gripper_right_ac_->waitForServer();
    ROS_INFO("Connected!");
    
    moveGripper("left", "close");
    moveGripper("right", "close");

    //! Location markers
    location_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/restaurant/location_markers", 10);

    //! Switch on perception
    pein_client_ = nh.serviceClient<perception_srvs::StartPerception>("/start_perception");
    vector<string> modules;
    modules.push_back("ppl_detection");
    if (!togglePein(modules))
    {
        ros::Duration wait(1.0);
        wait.sleep();
        if (!togglePein(modules))
        {
            ROS_ERROR("No ppl detection possible, end of challenge");
            return 1;
        }
    }

    //! Start speech recognition
    speech_recognition_client_ = nh.serviceClient<tue_pocketsphinx::Switch>("/pocketsphinx/switch");
    speech_recognition_client_.waitForExistence();
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 10, speechCallback);
    startSpeechRecognition();
    ROS_INFO("Started speech recognition");

    //! Clients for reasoner and WIRE for objects
    ROS_INFO("Connecting to WIRE...");
    wire::Client client;
    ROS_INFO("Connected!");
    ROS_INFO("Connecting to reasoner...");
    reasoner_client = new psi::Client("reasoner");
    ROS_INFO("Connected!");

    //! Client that allows reseting WIRE
    reset_wire_client_ = nh.serviceClient<std_srvs::Empty>("/wire/reset");
    ROS_INFO("Service /wire/reset");

    //! Connect to grab machine
    grab_machine_client_ = nh.serviceClient<challenge_restaurant::SmachStates>("/smach_states");

    //! Wait for start signal
    speech_client_.waitForExistence(ros::Duration(5.0));
    amigoSpeak("I will start when you say continue");
    double t1 = ros::Time::now().toSec();
    speech_interpreter::GetInfo srv_test;
    srv_test.request.n_tries = 1;
    srv_test.request.time_out = 10.0;
    srv_test.request.type = "continue_confirm";
    string answer = "";
    while (answer != "continue" &&
           ros::Time::now().toSec() - t1 < 60.0) {
        if (speech_client_.call(srv_test)) {
            answer = srv_test.response.answer;
        }
    
        ROS_INFO("Currently, answer =  %s", answer.c_str());
    }
    ROS_INFO("Waited %f [s]", ros::Time::now().toSec()-t1);

    //! Administration
    pbl::PDF guide_pos;
    
    //! Always clear the world model
    std_srvs::Empty srv;
    if (reset_wire_client_.call(srv))
    {
        ROS_INFO("Cleared world model");
    } else {
        ROS_ERROR("Failed to clear world model");
    }

    ROS_INFO("Found guide with position %s in frame \'%s\'", guide_pos.toString().c_str(), NAVIGATION_FRAME.c_str());
    amigoSpeak("Hi guide, can you please show me the locations");

    //! Follow guide
    freeze_amigo_ = false;
    ros::Rate follow_rate(FOLLOW_RATE);
    while (ros::ok() && state  == 0)
    {

        ros::spinOnce();

        //! Get objects from the world state
        follow_rate.sleep();
    }

    // PART II: Get orders

    // Reset world model
    reset_wire_client_.call(srv);

    // Switch off perception
    modules.clear();
    if (!togglePein(modules))
    {
        ROS_WARN("Perception not switched off");
    }

    // Take order
    stringstream sentence;
    sentence << "I would like to take " << N_ORDERS << " orders.";
    amigoSpeak(sentence.str());

    while (ros::ok() && state  == 1)
    {
        ros::spinOnce();
        follow_rate.sleep();

    }

    // Switch to move_base
    move_base_ac_ = new actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>("move_base", true);
    move_base_ac_->waitForServer();
    ROS_INFO("Connected to move_base server");

    // Do tasks
    unsigned int n_loops = 0;
    setRGBLights("blue");
    amigoRotate(7.5);
    while (state  == 2)
    {
        ros::spinOnce();
        ++n_loops;
        follow_rate.sleep();
    }

    while (state == 3)
    {

        // Then go back to order location
        if (location_map_.find("ordering_location") != location_map_.end()) {

            ROS_DEBUG("Ordering location is (%f,%f)",
                      location_map_["ordering_location"].getOrigin().x(), location_map_["ordering_location"].getOrigin().x());

            if (!moveTowardsPositionMap(location_map_["ordering_location"])) {
                ROS_WARN("Could not go back to ordering location");
                amigoSpeak("I can not find my way to the ordering location yet", false);
                amigoRotate(3.0);
            }
            else {
                amigoSpeak("I reached the ordering location");
                state = 4;
            }
        } else {
            ROS_WARN("No ordering location stored");
            amigoSpeak("I forgot the ordering location");
            state = 4;
        }

        follow_rate.sleep();


    }

    amigoSpeak("I did all I could. I am done with the challenge");

    //! TODO: when node is shut down, cancel goal by sending zero goal


    delete listener;
    delete head_ref_ac_;
    delete left_arm_ac_;
    delete right_arm_ac_;
    delete reasoner_client;
    delete gripper_left_ac_;
    delete gripper_right_ac_;

    return 0;
}
