// Ros
#include <ros/ros.h>

// Find ROS pkgs
#include <ros/package.h>

// Messages
#include <text_to_speech/Speak.h>
#include "tue_pocketsphinx/Switch.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"
#include "amigo_msgs/RGBLightCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

// Action client related
#include <actionlib/client/simple_action_client.h>
#include "robot_skill_server/ExecuteAction.h"
#include "amigo_head_ref/HeadRefAction.h"

// Interface WIRE
#include "wire_interface/Client.h"

// Services
#include "perception_srvs/StartPerception.h"
#include "std_srvs/Empty.h"

// Follower
#include "challenge_follow_me/Follower.h"

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
std::string robot_base_frame_ = "/amigo/base_link";

// Speech
ros::ServiceClient srv_speech_;                                                   // Communication: Service that makes AMIGO speak
ros::ServiceClient speech_recognition_client_;                                    // Communication: Client for starting / stopping speech recognition
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak

// Actuation
actionlib::SimpleActionClient<robot_skill_server::ExecuteAction>* ac_skill_server_;
actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>* ac_head_ref_;

// Visualization
ros::Publisher rgb_pub_;                                                          // Communicatino: Set color AMIGO
ros::Publisher location_marker_pub_;                                              // Communication: Marker publisher

// Follower
Follower* follower_;

// Tf
tf::TransformListener* listener_;												  // Tf listenter to obtain tf information to store locations

// Perception
ros::ServiceClient srv_pein_;

// Clear cost map
ros::ServiceClient srv_cost_map;

// Problib conversions
#include "problib/conversions.h"

// Administration: speech
bool speech_recognition_turned_on_ = false;
speech_state::SpeechState speech_state_ = speech_state::DRIVE;
double t_last_speech_cmd_ = 0;

// Adminstration: other
std::string current_clr_;
bool left_elevator_ = false;


std::string getSpeechStateName(speech_state::SpeechState ss)
{
    std::string name = "<unknown mode>";
    if (ss == speech_state::DRIVE) name = "DRIVE";
    else if (ss == speech_state::CONFIRM_LEAVE) name = "CONFIRM_LEAVE";

    return name;
}


bool transformPoseStamped(geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out, std::string frame_out)
{
    //! Transform goal to the map frame
    try
    {
        listener_->transformPose(frame_out, in, out);
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
    else if (color == "yellow")
    {
        clr_msg.r = 255;
        clr_msg.g = 255;
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





void speechCallbackGuideShort(std_msgs::String res)
{

    t_last_speech_cmd_ = ros::Time::now().toSec();

    //! Only consider non-empty answers
    std::string answer = res.data;
    ROS_INFO("Received command: %s", answer.c_str());
    if (answer.empty()) return;

    // ROBOT GOT A COMMAND
    if (speech_state_ == speech_state::DRIVE && answer == "amigoleave")
    {
        // stop robot
        follower_->pause();
        ROS_INFO("Paused follower!");

        // Get location and side
        amigoSpeak("Should I leave the elevator?");
        setRGBLights("green");
    }

    // RECEIVED THE COMMAND TO LEAVE THE ELEVATOR
    else if (speech_state_ == speech_state::CONFIRM_LEAVE)
    {
        // Get first word from the answer
        size_t position = answer.find_first_of(" ");
        if (position > 0 && position <= answer.size()) answer = std::string(answer.c_str(), position);

        // CONFIRMED
        if (answer == "yes")
        {
            //! Leave the elevator
            geometry_msgs::PoseStamped goal_pos;
            goal_pos.header.frame_id = robot_base_frame_;
            goal_pos.header.stamp = ros::Time::now();
            goal_pos.pose.position.x = -3.0;
            tf::Quaternion q;
            q.setRPY(0, 0, 3.14);
            goal_pos.pose.orientation.x = q.getX();
            goal_pos.pose.orientation.y = q.getY();
            goal_pos.pose.orientation.z = q.getZ();
            goal_pos.pose.orientation.w = q.getW();
            transformPoseStamped(goal_pos, goal_pos, "/map");
            moveBase(goal_pos.x, goal_pos.y, tf::getYaw(goal_pos.pose.orientation), 0.5, 25.0);

            //! Shutdown the speech
            sub_speech.shutdown();
            left_elevator_ = true;


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
    ac_skill_server_->waitForResult(ros::Duration(4.0));
    if(ac_skill_server_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WARN("Could not spindle position %f within 4 [s]", std_spindle_pos);
        return false;
    }

    return true;
}



bool moveBase(double x, double y, double theta, double goal_radius = 0.1, dt = 3.0)
{
    //double t_start_move = ros::Time::now().toSec();

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
        ROS_WARN("Could not reach base pose within 60 [s]");
        // Administration
        x_last_ = x;
        y_last_ = y;
        return false;
    }

    ROS_INFO("Reached base goal after %f [s].", ros::Time::now().toSec()-t_send_goal);

    // Administration
    x_last_ = x;
    y_last_ = y;
    return true;

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


bool togglePein(std::vector<std::string> modules) {

    perception_srvs::StartPerception pein_srv;

    //! Add required modules
    if (modules.empty())
    {
        // Turn off perception
        pein_srv.request.modules.push_back("");
    }
    else
    {
        // Add all modules
        std::vector<std::string>::const_iterator it = modules.begin();
        for (; it != modules.end(); ++it)
        {
            pein_srv.request.modules.push_back(*it);
        }
    }

    //! Toggle perception
    bool ok = srv_pein_.call(pein_srv);

    //! Feedback to the user
    if (ok) ROS_INFO("Switched on pein_modules:");
    else ROS_WARN("Could not switch on pein_modules:");
    std::vector<std::string>::const_iterator it = modules.begin();
    for (; it != modules.end(); ++it) ROS_INFO("\t%s", it->c_str());

    return ok;

}



void restartSpeechIfNeeded()
{
    if (ros::Time::now().toSec() - t_last_speech_cmd_ > 10.0)
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







int main(int argc, char **argv) {

    ros::init(argc, argv, "test_speech_restaurant");
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
    //ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 1, speechCallbackGuide);
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 1, speechCallbackGuideShort);

    //! Skill server action client
    ROS_INFO("Connecting to the skill server...");
    ac_skill_server_ = new actionlib::SimpleActionClient<robot_skill_server::ExecuteAction>("/amigo/execute_command", true);
    ac_skill_server_->waitForServer();
    ROS_INFO("Connected!");

    //! Reset spindle
    resetSpindlePosition();

    //! Head ref action client
    ROS_INFO("Connecting to head ref action server...");
    ac_head_ref_ = new actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>("head_ref_action", true);
    ac_head_ref_->waitForServer();
    ROS_INFO("Connected!");

    //! Clear cost map interface
    srv_cost_map = nh.serviceClient<tue_pocketsphinx::Switch>("/move_base_3d/reset");
    srv_cost_map.waitForExistence(ros::Duration(3.0));
    std_srvs::Empty empty_srv;
    if (!srv_cost_map.exists() && !srv_cost_map.call(empty_srv)) ROS_WARN("Cannot clear the cost map");

    //! Perception
    srv_pein_ = nh.serviceClient<perception_srvs::StartPerception>("/start_perception");

    //! Clearing the world model
    ros::ServiceClient reset_wire_client = nh.serviceClient<std_srvs::Empty>("/wire/reset");
    std_srvs::Empty srv;
    if (!reset_wire_client.call(srv)) ROS_WARN("Failed to clear world model");

    ///////////////// GUIDING PHASE //////////////////////////////////////////////////////////////////////////////////////////////////

    //! Start follower
    follower_ = new Follower(nh, robot_base_frame_, false);
    if (!follower_->start())
    {
        ROS_ERROR("Could not start the follower!");
        return -1;
    }

    //! Wait for operator
    bool operator_found = false;
    ros::Rate loop_rate(25);
    while (!operator_found)
    {
        operator_found = follower_->update();
        loop_rate.sleep();

    }

    stopSpeechRecognition();
    startSpeechRecognition();
    setRGBLights("green");
    t_last_speech_cmd_ = ros::Time::now().toSec();
    ROS_INFO("Started speech recognition");

    //! Start Following
    bool drive = false;
    double t_start_no_move = 0;
    while (ros::ok())
    {
        //! Get information from topics
        ros::spinOnce();

        //! Update the follower
        bool non_zero_vel = follower_->update();

        //! If the robot did not move for a while, try moving using move_base_3d
        if (!non_zero_vel)
        {
            if (drive) t_start_no_move = ros::Time::now().toSec();
            ROS_DEBUG("Robot does not move");
            drive = false;

            if (ros::Time::now().toSec() - t_start_no_drive > 5.0)
            {

                if (left_elevator_)
                {
                    // AFTER THE ELEVATOR: move forward
                    follower_->pause();
                    // @todo: move 2 [m] forward
                    follower_->reset();


                }
                else
                {
                    // BEFORE THE ELEVATOR: just try to move
                    ROS_WARN("Robot did not move for %f [s], trying move_base_3d to plan around obstacle", ros::Time::now().toSec() - t_start_no_drive);
                    double x = 0, y = 0, phi = 0;
                    follower_->getCurrentOperatorPosition(x, y, phi, robot_base_frame_);
                    moveBase(x, y, phi, 0.2, 2.0);
                }
            }

            // @todo: after leaving the elevator another str

        }
        else drive = true;

        //! To ensure speech keeps working
        restartSpeechIfNeeded();

        //! Wait
        loop_rate.sleep();
    }

    delete ac_skill_server_;
    delete ac_head_ref_;

    return 0;
}

