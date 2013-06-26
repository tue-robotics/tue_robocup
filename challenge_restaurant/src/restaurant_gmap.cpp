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
#include <text_to_speech_philips/Speak.h>
#include "tue_pocketsphinx/Switch.h"

// Services
#include <std_srvs/Empty.h>
#include "perception_srvs/StartPerception.h"
#include "speech_interpreter/GetInfo.h"
#include "challenge_restaurant/SmachStates.h"

// Problib conversions
#include "problib/conversions.h"

// Simple path planner
#include "tue_carrot_planner/carrot_planner.h"


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
CarrotPlanner* planner_;
double t_no_meas_ = 0;                                                            // Bookkeeping: determine how long guide is not observed
double t_last_check_ = 0;                                                         // Bookkeeping: last time guide position was checked
double last_var_guide_pos_ = -1;                                                  // Bookkeeping: last variance in x-position guide
bool freeze_amigo_ = false;                                                       // Bookkeeping: true if stop command is confirmed
bool candidate_freeze_amigo_ = false;                                             // Bookkeeping: true if robot is asked to stop
bool stored_location = false;                                                     // Bookkeeping: true if robot another location must be learned
bool finished = false;                                                            // Bookkeeping: true if robot must go to ordering location
int state = 0;                                                                    // Bookkeeping: state
unsigned int n_locations_deliver_ = 0;                                            // Bookkeeping: number of locations learned
unsigned int n_shelves_ = 0;                                                      // Bookkeeping: number of shelves learned
double t_freeze_ = 0;                                                             // Bookkeeping: time the robot is frozen
bool speech_recognition_on = false;                                               // Bookkeeping: speech switched on/off

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


void setRGBLights(string color) {

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


trajectory_msgs::JointTrajectoryPoint setArmReference(double r0, double r1, double r2, double r3, double r4, double r5, double r6) {

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
    } else if (pose == "give") {
        moveArm(arm, "carry");
        arm_ref.trajectory.points.push_back(setArmReference(-0.93 , 1.06 , 0.07 , 0.91 , -1.02 , -0.19 , -0.36));
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


void resetRGBLights() {
    //! Send color command
    amigo_msgs::RGBLightCommand rgb_cmd;
    rgb_cmd.show_color.data = false;
    rgb_pub_.publish(rgb_cmd);
}



void amigoRotate(double delta) {

    ROS_INFO("Rotate for %f [s]...", delta);

    pbl::Matrix cov(3,3);
    cov.zeros();
    pbl::Gaussian pos = pbl::Gaussian(pbl::Vector3(-1, 0, 0), cov);
    const unsigned int freq = 20;
    unsigned int n_sleeps_total = delta*freq;
    ros::Duration pause(1.0/(double)freq);
    unsigned int n_sleeps = 0;

    while (n_sleeps < n_sleeps_total) {
        moveTowardsPosition(pos, 2);
        pause.sleep();
        ++n_sleeps;
    }

    ROS_INFO("Done rotating");
}


bool startSpeechRecognition() {
    std::string knowledge_path = ros::package::getPath("tue_knowledge");
    if (knowledge_path == "") {
        return false;
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
    if (toggle_speech) stopSpeechRecognition();

    //! Call speech service
    text_to_speech_philips::Speak speak;
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



/**
 * @brief findGuide, detects person in front of robot, empties WIRE and adds person as guide
 * @param client used to query from/assert to WIRE
 */
bool findGuide(wire::Client& client, bool lost = true) {

    //! It is allowed to call the guide once per section (points for the section will be lost)
    if (lost) {
        setRGBLights("yellow");
        amigoSpeak("Wait for me guide");
        setRGBLights("yellow");
    }

    //! Give the guide some time to move to the robot
    ros::Duration wait_for_guide(1.0);
    wait_for_guide.sleep();

    //! Vector with candidate guides
    vector<pbl::Gaussian> vector_possible_guides;

    //! See if the a person stands in front of the robot
    double t_start = ros::Time::now().toSec();
    ros::Duration dt(1.0);
    bool no_guide_found = true;
    while (ros::Time::now().toSec() - t_start < WAIT_TIME_GUIDE_MAX && no_guide_found) {

        //! Get latest world state estimate
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);

        //! Iterate over all world model objects and look for a person in front of the robot
        for(vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
            wire::PropertySet& obj = *it_obj;
            const wire::Property& prop_label = obj.getProperty("class_label");
            if (prop_label.isValid() && prop_label.getValue().getExpectedValue().toString() == "person") {

                //! Check position
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid()) {

                    //! Get position of potential guide
                    pbl::PDF pos = prop_pos.getValue();
                    pbl::Gaussian pos_gauss(3);
                    if (pos.type() == pbl::PDF::GAUSSIAN) {
                        pos_gauss = pbl::toGaussian(pos);

                    } else {
                        ROS_INFO("restaurant_simple (findGuide): Position person is not a Gaussian");
                    }

                    //! Check if the person stands in front of the robot
                    if (pos_gauss.getMean()(0) < 2.5 && pos_gauss.getMean()(0) > 0.3 && pos_gauss.getMean()(1) > -0.4 && pos_gauss.getMean()(1) < 0.4) {
                        vector_possible_guides.push_back(pos_gauss);
                        ROS_INFO("Found candidate guide at (x,y) = (%f,%f)", pos_gauss.getMean()(0), pos_gauss.getMean()(1));

                    }
                }

            }

        }

        //! Found at least one candidate guide
        if (!vector_possible_guides.empty()) {

            //! Position candidate guide
            pbl::Gaussian pos_guide(3);

            //! Find person that has smallest Eucledian distance to point in front of the robot
            if (vector_possible_guides.size() > 1) {

                double dist_min = 0.0;
                int i_best = 0;

                for (unsigned int i = 0; i < vector_possible_guides.size(); ++i)
                {
                    double dx = TYPICAL_GUIDE_X - vector_possible_guides[i].getMean()(0);
                    double dy = TYPICAL_GUIDE_Y - vector_possible_guides[i].getMean()(1);
                    double dist = sqrt(dx*dx+dy*dy);
                    if (i == 0 || dist < dist_min)
                    {
                        dist_min = dist;
                        i_best = i;
                    }
                }

                pos_guide = vector_possible_guides[i_best];

            } else {
                pos_guide = vector_possible_guides[0];
            }

            ROS_INFO("Found guide at (x,y) = (%f,%f)", pos_guide.getMean()(0), pos_guide.getMean()(1));

            //! Reset
            last_var_guide_pos_ = -1;
            t_last_check_ = ros::Time::now().toSec();

            //! Evidence
            wire::Evidence ev(t_last_check_);

            //! Set the position
            ev.addProperty("position", pos_guide, NAVIGATION_FRAME);

            //! Name must be guide
            pbl::PMF name_pmf;
            name_pmf.setProbability("guide", 1.0);
            ev.addProperty("name", name_pmf);

            //! Reset the world model
            std_srvs::Empty srv;
            if (reset_wire_client_.call(srv)) {
                ROS_INFO("Cleared world model");
            } else {
                ROS_ERROR("Failed to clear world model");
            }

            //! Assert evidence to WIRE: multiple times (just to be sure)
            for (unsigned int dummy = 0; dummy < 3; ++dummy) {
                client.assertEvidence(ev);
            }

            no_guide_found = false;

            amigoSpeak("I found my guide.");

            ros::Duration safety_delta(0.5);
            safety_delta.sleep();
            return true;

        }

        dt.sleep();
    }

    amigoSpeak("I did not find my guide yet");
    return false;

}


/**
* @brief getPositionGuide
* @param objects input objects received from WIRE
* @param pos position of the guide (output)
* @return bool indicating whether or not the guide was found
*/
bool getPositionGuide(vector<wire::PropertySet>& objects, pbl::PDF& pos) {

    //! Iterate over all world model objects
    for(vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        wire::PropertySet& obj = *it_obj;
        const wire::Property& prop_name = obj.getProperty("name");
        if (prop_name.isValid()) {

            //! Check if the object represents the guide
            if (prop_name.getValue().getExpectedValue().toString() == "guide") {
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid()) {

                    //! Get the guide's position
                    pos = prop_pos.getValue();

                    //! Get position covariance
                    pbl::Gaussian pos_gauss(3);
                    if (pos.type() == pbl::PDF::GAUSSIAN) {
                        pos_gauss = pbl::toGaussian(pos);

                    } else if (pos.type() == pbl::PDF::MIXTURE) {
                        pbl::Mixture mix = pbl::toMixture(pos);

                        double w_best = 0;

                        for (unsigned int i = 0; i < mix.components(); ++i) {
                            pbl::PDF pdf = mix.getComponent(i);
                            pbl::Gaussian G = pbl::toGaussian(pdf);
                            double w = mix.getWeight(i);
                            if (G.isValid() && w > w_best) {
                                pos_gauss = G;
                                w_best = w;
                            }
                        }
                    }
                    pbl::Matrix cov = pos_gauss.getCovariance();

                    //ROS_INFO("Guide has variance %f, last variance is %f", cov(0,0), last_var_guide_pos_);


                    //! Check if guide position is updated (initially negative)
                    if (cov(0,0) < last_var_guide_pos_ || last_var_guide_pos_ < 0) {
                        last_var_guide_pos_ = cov(0,0);
                        t_no_meas_ = 0;
                        ROS_DEBUG("Time since last update was %f", ros::Time::now().toSec()-t_last_check_);
                        t_last_check_ = ros::Time::now().toSec();
                    } else {

                        //! Uncertainty increased: guide out of side
                        last_var_guide_pos_ = cov(0,0);
                        if (!freeze_amigo_ && !candidate_freeze_amigo_) {
                            t_no_meas_ += (ros::Time::now().toSec() - t_last_check_);
                        } else {
                            t_last_check_ = ros::Time::now().toSec();
                        }
                        if (t_no_meas_ > 1.5) ROS_DEBUG("%f [s] without position update guide: ", t_no_meas_);
                        //ROS_INFO("%f [s] without position update guide: ", t_no_meas_);

                        //! Position uncertainty increased too long: guide lost
                        if (t_no_meas_ > TIME_OUT_GUIDE_LOST) {
                            ROS_INFO("I lost my guide");
                            return false;
                        }
                    }

                    t_last_check_ = ros::Time::now().toSec();

                    return true;

                } else {
                    ROS_WARN("Found a guide without valid position attribute");
                }
            }
        }
    }

    //! If no guide was found, return false
    return false;
}




/**
* @brief getPositionGuide
* @param objects input objects received from WIRE
* @param pos position of the guide (output)
* @return bool indicating whether or not the guide was found
*/
map<string, pair<geometry_msgs::Point, string> > getWorldModelObjects(vector<wire::PropertySet>& objects) {

    // Map storing the copy of the world model
    map<string, pair<geometry_msgs::Point, string> > world_model_copy;

    // Needed to determine instance with highest probability if multiple instances of same class
    map<string, double> class_prob_map;

    //! Iterate over all world model objects
    for(vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        wire::PropertySet& obj = *it_obj;
        const wire::Property& prop_label = obj.getProperty("class_label");
        if (prop_label.isValid()) {

            string class_label = prop_label.getValue().getExpectedValue().toString();
            double prob = pbl::toPMF(prop_label.getValue()).getProbability(prop_label.getValue().getExpectedValue());

            ROS_INFO("Found %s!", class_label.c_str());

            const wire::Property& prop_pos = obj.getProperty("position");
            if (prop_pos.isValid()) {

                //! Get the position
                pbl::PDF pos = prop_pos.getValue();
                pbl::Vector pos_obj = pos.getExpectedValue().getVector();
                ROS_INFO("Position of %s is (%f,%f,%f)", class_label.c_str(),  pos_obj(0), pos_obj(1), pos_obj(2));

                geometry_msgs::Point pos_store;
                pos_store.x = pos_obj(0);
                pos_store.y = pos_obj(1);
                pos_store.z = pos_obj(2);


                //! Store object in world model
                if (world_model_copy.find(class_label) == world_model_copy.end()) {

                    // If class is not yet in world model, store
                    world_model_copy[class_label] = make_pair(pos_store, obj.getID());

                }
                // If class is present in copy already, store instance with highest probability
                else {

                    amigoSpeak("I see multiple instances of " + class_label + ", I will remember the most probable one.");

                    // Check/correct administration
                    if (class_prob_map.find(class_label) == class_prob_map.end()) {
                        ROS_WARN("Something went wrong in the administration");
                        class_prob_map[class_label] = 0;
                    }


                    if (class_prob_map[class_label] >= prob) {

                        world_model_copy[class_label] = make_pair(pos_store, obj.getID());
                        class_prob_map[class_label] = prob;
                    }
                }
            }


        }
    }

    return world_model_copy;

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


/**
 * @brief speechCallback
 * @param res
 */
void speechCallback(std_msgs::String res) {

    ROS_INFO("Received command: %s", res.data.c_str());
    
    if (res.data == "left" || res.data == "right" || res.data == "front") {
        ROS_WARN("Received old command");
        return;
    }        

    //// AMIGO STOP
    if (res.data == "amigostop") {
        candidate_freeze_amigo_ = true;
        amigoSpeak("Do you want me to stop?");
        t_freeze_ = ros::Time::now().toSec();
    }
    //// AMIGO STOP: YES
    // Amigo heard amigostop and answer is confirmed
    else if (candidate_freeze_amigo_ &&  res.data == "yes" && !finished) {
        freeze_amigo_ = true;
        candidate_freeze_amigo_ = false;
        amigoSpeak("Is this location a shelf?");
    }
    // All locations are learned and Amigo is at ordering location
    else if (candidate_freeze_amigo_ &&  res.data == "yes" && finished) {
        candidate_freeze_amigo_ = false;
        amigoSpeak("I will remember the ordering location");

        // Save ordering location
        tf::StampedTransform trans;
        listener->lookupTransform("/map", "/base_link", ros::Time(0), trans);

        location_map_["ordering_location"] = trans;
        ROS_INFO("Saved the ordering location with transform parameters : [%f,%f]",
                 trans.getOrigin().x(), trans.getOrigin().y() );

        // Publish marker
        visualization_msgs::MarkerArray marker_array;
        createMarkerWithLabel("Ord. loc.",trans, 0, 0, 1, marker_array);
        location_marker_pub_.publish(marker_array);

        freeze_amigo_ = false;
        state = 1;
    }
    //// AMIGO STOP: NO
    // All locations are learned and Amigo is at ordering location
    else if (candidate_freeze_amigo_ &&  res.data != "yes" && res.data != "") {
        candidate_freeze_amigo_ = false;
        amigoSpeak("I misunderstood, I will follow you");
    }

    //// AMIGO ASKED: PICKUP LOCATION
    else if (freeze_amigo_) {


        // Get position
        tf::StampedTransform trans;
        listener->lookupTransform("/map", "/base_link", ros::Time(0), trans);

        // Create marker
        visualization_msgs::MarkerArray marker_array;

        // Get location:
        std::stringstream location_name;

        // Marker color
        double r = 0, g = 0, b = 0;

        // Answer is yes: indeed a shelf
        if (res.data == "yes") {

            // Administration
            r = 1;
            ++n_shelves_;
            location_name << "shelf" << n_shelves_;

            // Ask for a side
            amigoSpeak("Where do I find the shelf?");

        }
        // Answer is no: not a shelf, but a delivery location
        else {

            // Administration
            g = 1;
            ++n_locations_deliver_;
            location_name << "delivery location" << n_locations_deliver_;

            // Ask for a side
            amigoSpeak("Where do I find the delivery location?");
        }

        // Get side from speech interpreter (left/right/front)
        resetRGBLights();
        string answer = "";
        if (callInterpreter("side", answer)) {
            ROS_WARN("Service call to speech interpreter fail: no side available");
        }

        // Determine corresponding theta
        double theta = 0;
        if (answer == "left") theta = 1.57;
        else if (answer == "right") theta = -1.57;

        // Set updated orientation: adding angles corresponds to multiplying the quaternions
        tf::Quaternion q = trans.getRotation();
        ROS_INFO("Before update orientation is (%f,%f,%f,%f)", q.getW(), q.getX(), q.getY(), q.getZ());
        tf::Quaternion offset;
        offset.setRPY(0, 0, theta);
        q *= offset;
        trans.setRotation(q);
        ROS_INFO("After update orientation is (%f,%f,%f,%f)", q.getW(), q.getX(), q.getY(), q.getZ());


        // Store location
        location_map_[location_name.str()] = trans;
        ROS_INFO("Saved the %s with transform parameters : [%f,%f]",
                 location_name.str().c_str() ,trans.getOrigin().x(), trans.getOrigin().y());

        // Publish marker
        createMarkerWithLabel(location_name.str(), trans, r, g, b, marker_array);
        location_marker_pub_.publish(marker_array);


        //! Inform user
        string sentence = "I will call this location " + location_name.str();
        amigoSpeak(sentence);
        amigoSpeak("Do you want to learn another location?");

        //! Sleep (amigoSpeak is blocking)
        //ROS_INFO("I will sleep");
        //ros::Duration delta(7.0);
        //delta.sleep();
        //ROS_INFO("Sleep is over");

        // Administration
        freeze_amigo_ = false;
        stored_location = true;

    }
    //// AMIGO MUST LEARN ANOTHER LOCATION
    else if (stored_location &&  res.data == "yes") {
        finished = false;
        stored_location = false;
        amigoSpeak("Alright");

        // Rotate to clear guide from the map
        //amigoRotate(1.5);
    }
    //// AMIGO WILL GO TO ORDERING LOCATION
    else if (stored_location &&  res.data != "yes" && res.data != "") {
        ROS_INFO("I heard %s", res.data.c_str());
        finished = true;
        stored_location = false;
        amigoSpeak("Certainly. Please guide me to the ordering location?");

        // Rotate to clear guide from the map
        //amigoRotate(1.5);
    }
    /////

    // always immediately start listening again
    startSpeechRecognition();

    // Robot will not move, this avoids lost operator after conversation
    t_last_check_ = ros::Time::now().toSec();

}






/**
 * @brief moveTowardsPosition Let AMIGO move from its current position towards the given position
 * @param pos target position
 * @param offset, in case the position represents the guide position AMIGO must keep distance
 */
void moveTowardsPosition(pbl::PDF& pos, double offset) {

    pbl::Vector pos_exp = pos.getExpectedValue().getVector();

    //! End point of the path is the given position
    geometry_msgs::PoseStamped end_goal;
    end_goal.header.frame_id = NAVIGATION_FRAME;
    double theta = atan2(pos_exp(1), pos_exp(0));
    tf::Quaternion q;
    q.setRPY(0, 0, theta);

    //amigo_msgs::head_ref goal;
    //goal.head_pan = theta;
    //goal.head_tilt = 0.0;
    //head_ref_pub_.publish(goal);

    //! Set orientation
    end_goal.pose.orientation.x = q.getX();
    end_goal.pose.orientation.y = q.getY();
    end_goal.pose.orientation.z = q.getZ();
    end_goal.pose.orientation.w = q.getW();

    //! Incorporate offset in target position
    double full_distance = sqrt(pos_exp(0)*pos_exp(0)+pos_exp(1)*pos_exp(1));
    double reduced_distance = std::max(0.0, full_distance - offset);
    end_goal.pose.position.x = pos_exp(0) * reduced_distance / full_distance;
    end_goal.pose.position.y = pos_exp(1) * reduced_distance / full_distance;
    end_goal.pose.position.z = 0;

    if (t_no_meas_ < 1.0) {
        planner_->MoveToGoal(end_goal);
        ROS_DEBUG("Executive: Move base goal: (x,y,theta) = (%f,%f,%f) - red. and full distance: %f and %f", end_goal.pose.position.x, end_goal.pose.position.y, theta, reduced_distance, full_distance);
    } else {
        ROS_DEBUG("No guide position update: robot will not move");
    }

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


void checkOrderWithWorldModelAtShelf(map<string, pair<geometry_msgs::Point, string> >& world_model_copy, tf::StampedTransform tf_loc_shelf) {

    map<string, pair<geometry_msgs::Point, string> >::iterator it_world_model = world_model_copy.begin();

    // Iterate over world model objects
    for (; it_world_model != world_model_copy.end(); ++it_world_model) {

        // Check current object
        map<int, pair<string, string> >::iterator it_order = order_map_.begin();
        for (; it_order != order_map_.end(); ++it_order) {

            // Check if class labels are the same and the ID is not equal to object_delivered
            if (it_order->second.first == it_world_model->first && it_world_model->second.second != "object_delivered") {

                // Make sure AMIGO moves towards the shelf
                if (!moveTowardsPositionMap(tf_loc_shelf)) {
                    ROS_WARN("Could not reach the shelf");
                    amigoSpeak("I cannot reach the shelf, can you let me pass?");
                    ros::Duration d(2.0);
                    d.sleep();
                    if (!moveTowardsPositionMap(tf_loc_shelf)) {
                        ROS_WARN("Failed second attempt to reach shelf too");
                        //at_loc = false;
                    }
                }

                ROS_INFO("Found the object for order %d!", it_order->first);
                amigoSpeak("I found " + it_world_model->first);
                amigoSpeak("I will bring it to delivery location " + it_order->second.second);

                // Reset head
                moveHead(0.0, 0.0);

                // Print information
                stringstream location_name;
                location_name << "delivery location" << it_order->second.second;
                tf::StampedTransform tf_loc_object = location_map_[location_name.str()];
                ROS_INFO("Bring %s to location %s: (%f,%f)",
                         it_world_model->first.c_str(), it_order->second.second.c_str(), tf_loc_object.getOrigin().x(), tf_loc_object.getOrigin().y());


                // retract all current_object/1 facts from reasoner (prolog call: retractall(current_object(X)) )
                reasoner_client->query(psi::Compound("retractall", psi::Compound("current_object", psi::Variable("X"))));

                // assert current object id to reasoner (prolog call: assert(current_object(ID)) )
                reasoner_client->query(psi::Compound("assert", psi::Compound("current_object", psi::Constant(it_world_model->second.second))));

                // Call service to python grab machine
                string arm_used;
                challenge_restaurant::SmachStates ss_srv;
                ss_srv.request.state = "grasp";
                bool suc = false;
                if (!grab_machine_client_.call(ss_srv)) {
                    ROS_WARN("Service call to grab_machine failed");
                } else if (ss_srv.response.outcome != "Succeeded") {
                    ROS_WARN("Service call returned %s", ss_srv.response.outcome.c_str());
                } else {
                    arm_used = "left";
                    suc = true;
                }

                // If grabbing the object did not succeed, ask user to hand over
                if (!suc) {

                    amigoSpeak("Please handover the object, I am not able to grasp");
                    if (!moveArm("right", "give")) {
                        amigoSpeak("I can not move my right arm the way I want it");
                        arm_used = "right";
                    }

                    // Open gripper
                    if (!arm_used.empty()) {
                        if (!moveGripper(arm_used, "open")) {
                            amigoSpeak("I am sorry but I cannot open my gripper");
                            amigoSpeak("I will drive to the delivery location without the object");
                        } else {
                            ros::Duration wait_for_object(3.0);
                            wait_for_object.sleep();
                            amigoSpeak("I will close my gripper now");
                            if (!moveGripper(arm_used, "close")) {
                                amigoSpeak("I am sorry but I cannot close my gripper");
                                amigoSpeak("I will drive to the delivery location without the object");
                            }
                        }
                    }

                }

                // To make sure the arm is in a carrying position
                if (!moveArm(arm_used, "carry")) {
                    amigoSpeak("I cannot move my arms the way I want it, I will drive with my arm like this");
                }

                // Make sure other arm is in drive position
                if (arm_used == "left")
                {
                    moveArm("right", "drive");
                }
                else if (arm_used == "right")
                {
                    moveArm("right", "drive");
                }

                // Move to delivery location
                bool at_loc = true;
                if (!moveTowardsPositionMap(tf_loc_object)) {
                    ROS_WARN("Could not reach the delivery location");
                    amigoSpeak("I cannot reach the delivery location, can you let me pass?");
                    ros::Duration d(2.0);
                    d.sleep();
                    if (!moveTowardsPositionMap(tf_loc_object)) {
                        ROS_WARN("Failed second attempt to reach the delivery location too");
                        at_loc = false;
                    }
                }

                if (at_loc) {

                    // Arrived at delivery location
                    ROS_INFO("I finished task: %d!", it_order->first);
                    amigoSpeak("I am at delivery location " + it_order->second.second);
                }
                else {
                    amigoSpeak("I cannot get to the delivery location");
                }

                // Deliver the object by opening the gripper
                if (!arm_used.empty()) {

                    if (!moveArm(arm_used, "give")) {
                        amigoSpeak("I cannot move my arms the way I want it");
                    }

                    amigoSpeak("I will open my gripper, can you please take the " + it_world_model->first);
                    ros::Duration sleep_to_be_sure(2.0);
                    sleep_to_be_sure.sleep();
                    if (!moveGripper(arm_used, "open")) {
                        amigoSpeak("I am sorry but I cannot open my gripper");
                    }

                    // First go back to carry to make movement a bit safer
                    if (!moveArm(arm_used, "carry")) {
                        moveArm(arm_used, "drive");
                        amigoSpeak("I cannot move my arms the way I want it");
                    }
                }

                // Update the maps
                order_map_[it_order->first] = make_pair<string, string>("Finished", "Finished");
                it_world_model->second.second = "object_delivered";

            }

        }
    }

    // Done with checking at current shelf

    // Reset world model (since it is in base_link) and its copy
    world_model_copy.clear();
    std_srvs::Empty srv;
    if (reset_wire_client_.call(srv)) {
        ROS_INFO("Cleared world model");
    } else {
        ROS_ERROR("Failed to clear world model");
    }

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



bool moveHead(double pan, double tilt) {

    //! Add head reference action
    amigo_head_ref::HeadRefGoal head_ref;
    head_ref.goal_type = 1; // 1: pan tilt, 0 keep tracking
    head_ref.pan = pan;
    head_ref.tilt = tilt;
    head_ref_ac_->sendGoal(head_ref);
    head_ref_ac_->waitForResult(ros::Duration(5.0));

    if(head_ref_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("Head could not reach target position");
        return false;
    }

    return true;

}

bool callInterpreter(string type, string& answer) {

    // Stop challenge specific speech recognition
    stopSpeechRecognition();

    // Point mic towards guide
    //moveHead(0.1, 0.0);

    // Feedback
    bool succeeded = false;

    // Cal speech interpreter
    answer = "";
    speech_interpreter::GetInfo srv_test;
    srv_test.request.n_tries = 2;
    srv_test.request.time_out = 20;
    srv_test.request.type = type;
    if (speech_client_.call(srv_test)) {
        answer = srv_test.response.answer;
        succeeded = true;
    }

    // Start speech recognition again
    startSpeechRecognition();

    // Reset head
    //moveHead(0.0, 0.0);

    return succeeded;

}



int main(int argc, char **argv) {
    ros::init(argc, argv, "restaurant_simple");
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
    srv_speech_ =  nh.serviceClient<text_to_speech_philips::Speak>("/text_to_speech/speak");
    ROS_INFO("Publisher/service client for text to speech started");

    //! Head ref action client
    ROS_INFO("Connecting to head ref action server...");
    head_ref_ac_ = new actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>("head_ref_action", true);
    head_ref_ac_->waitForServer();
    ROS_INFO("Connected!");

    /// set the head to look down in front of AMIGO
    moveHead(0.1, 0.0);

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

    //! Planner
    double max_vel_lin = 0.5;                 // Default: 0.50
    double max_vel_ang = 0.4;                 // Default: 0.40
    double dist_to_wall = DISTANCE_GUIDE-0.1; // Default: 0.65
    planner_ = new CarrotPlanner("restaurant_carrot_planner", max_vel_lin, max_vel_ang, dist_to_wall);
    ROS_INFO("Carrot planner instantiated");

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

    //! Start speech recognition
    speech_recognition_client_ = nh.serviceClient<tue_pocketsphinx::Switch>("/pocketsphinx/switch");
    speech_recognition_client_.waitForExistence();
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 1, speechCallback);
    startSpeechRecognition();
    ROS_INFO("Started speech recognition");

    //! Always clear the world model
    std_srvs::Empty srv;
    if (reset_wire_client_.call(srv)) {
        ROS_INFO("Cleared world model");
    } else {
        ROS_ERROR("Failed to clear world model");
    }

    //! Connect to grab machine
    grab_machine_client_ = nh.serviceClient<challenge_restaurant::SmachStates>("/smach_states");

    //! Administration
    pbl::PDF guide_pos;

    amigoSpeak("I am looking for my guide");
    if (!findGuide(client, false)) {
        ROS_WARN("No guide can be found, try once more");
        if (!findGuide(client, false)) {
            ROS_ERROR("No guide found - fail");
            amigoSpeak("I cannot find a guide, I give up");
            return 0;
        }
    }

    ROS_INFO("Found guide with position %s in frame \'%s\'", guide_pos.toString().c_str(), NAVIGATION_FRAME.c_str());
    amigoSpeak("Can you please show me the locations");

    //! Follow guide
    freeze_amigo_ = false;
    ros::Rate follow_rate(FOLLOW_RATE);
    while(ros::ok() && state  == 0) {

        ros::spinOnce();

        //! Get objects from the world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);

        //! Check if the robot has to move
        if (candidate_freeze_amigo_ || freeze_amigo_) {

            setRGBLights("green");

            // Robot will not move, this avoids lost operator after conversation
            t_last_check_ = ros::Time::now().toSec();

            // Just follow
            if (candidate_freeze_amigo_) {
                ROS_DEBUG("Time out is %f", ros::Time::now().toSec() - t_freeze_);
                if (ros::Time::now().toSec() - t_freeze_ > 5) {
                    ROS_WARN("Time out");
                    candidate_freeze_amigo_ = false;
                    amigoSpeak("I will follow you");
                    setRGBLights("blue");
                }
            }

            //! Robot is waiting for the name and will then continue following
            pbl::Matrix cov(3,3);
            cov.zeros();
            pbl::PDF pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
            moveTowardsPosition(pos, 0);

            //! Reset head
            moveHead(0.0, 0.0);


        } else {

            setRGBLights("blue");

            //! Check for the (updated) guide position
            if (getPositionGuide(objects, guide_pos)) {

                //! Move towards guide
                // Bad: driving is not very smooth
                // Good: if the operator cannot be reached, e.g., because he is close but his position
                //       can't be updated either, the executive keeps sending a goal. Robot reacts, but
                //       if the robot doesn't observe the guide, it keeps on driving the wrong way
                if (t_no_meas_ < 1.5) { // makes driving less accurate, but avoids continuously sending a goal
                    moveTowardsPosition(guide_pos, DISTANCE_GUIDE);
                }


            } else {

                //! Lost guide
                // TODO Only find if candidate_freeze_amigo_ is false?
                bool found = findGuide(client);
                if (!found) {
                    ROS_WARN("Lost guide and failed to find one!");
                }
            }
        }

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

    while(ros::ok() && state  == 1) {
        ros::spinOnce();

        speech_interpreter::GetInfo srv;
        srv.request.n_tries = 3;
        srv.request.time_out = 40;

        for (unsigned int order_index = 0; order_index < N_ORDERS; ++order_index) {

            // Ask for a object class
            string obj_class = "";
            bool right_answer = false;
            string desired_object = "coke";

            // Ask for object class
            resetRGBLights();
            if (callInterpreter("object_classes", obj_class)) {

                // Check answer object class
                if (obj_class != "no_answer" && obj_class != "wrong_answer") {
                    //amigoSpeak("Which object would you like from this class?");

                    // Ask which object from class
                    resetRGBLights();
                    string obj_instance = "";
                    if (callInterpreter(obj_class, obj_instance)) {
                        if (obj_instance != "no_answer" && obj_instance != "wrong_answer") {
                            desired_object = obj_instance;
                            right_answer = true;
                        }

                    }
                }
            }

            if (!right_answer) {
                amigoSpeak("My mike does not work, I will bring a coke");
            }

            // Ask for a delivery location number
            right_answer = false;
            string desired_location = "one";
            amigoSpeak("To which delivery location should I bring the object?");

            // Ask for object class
            resetRGBLights();
            string given_loc = "";
            if (callInterpreter("numbers", given_loc)) {

                // Check answer object class
                if (given_loc != "no_answer" && given_loc != "wrong_answer") {
                    desired_location = given_loc;
                    right_answer = true;
                }
            }

            if (!right_answer) {
                amigoSpeak("My mike does not work, I will bring it to delivery location one");
            }


            // finished learning this task
            stringstream sent;
            sent << "Order " << order_index+1 << " is to bring the " << desired_object << " to delivery location " << desired_location;
            amigoSpeak(sent.str());
            ROS_INFO("Order %d: bring the %s to delivery location %s", order_index+1, desired_object.c_str(), desired_location.c_str());

            // Mapping from string to int ("one" -> "1")
            map<string, string> number_map;
            number_map["one"] = "1";
            number_map["two"] = "2";
            number_map["three"] = "3";

            // Check if delivery location exists (in location_map_)
            if (number_map.find(desired_location) != number_map.end()) {
                desired_location = number_map[desired_location];
            }
            else {
                amigoSpeak("My mike does not work, I will bring it to delivery location one");
                desired_location =  "1";
            }

            // Add order to the map
            order_map_[order_index] = make_pair<string, string>(desired_object, desired_location);

        }

        state = 2;


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
    while(state  == 2) {
        ros::spinOnce();

        map<string, tf::StampedTransform>::const_iterator it_locations = location_map_.begin();
        for (; it_locations != location_map_.end(); ++it_locations) {

            // If the current location is a shelf
            if (it_locations->first.find("shelf") != std::string::npos) {

                bool at_loc = true;

                // Go find the object
                if (!moveTowardsPositionMap(it_locations->second)) {
                    ROS_WARN("Could not reach the shelf");
                    amigoSpeak("I can not find my way to the shelf yet", false);
                    amigoRotate(3.5);

                    if (!moveTowardsPositionMap(it_locations->second)) {
                        ROS_WARN("Failed second attempt to reach shelf too");
                        amigoSpeak("I can not reach " + it_locations->first);
                        at_loc = false;
                    }
                }

                if (at_loc) {
                    amigoSpeak("I am at " + it_locations->first + ". I will now see if I can find objects");

                    //! Look down
                    moveHead(0.0, 0.35);

                    //! Clear world model and switch on perception
                    reset_wire_client_.call(srv);
                    modules.clear();
                    modules.push_back("template_matching");
                    perception_srvs::StartPerception pein_srv_temp_on;
                    pein_srv_temp_on.request.modules.push_back("template_matching");
                    if (!togglePein(modules))
                    {
                        amigoSpeak("I could not switch on template matching");
                    }
                    else
                    {

                        // Template matching needs some time
                        ros::Duration wait(5.0);
                        wait.sleep();

                        // Switch off perception
                        modules.clear();
                        if (!togglePein(modules))
                        {
                            amigoSpeak("I could not switch off template matching");
                        }

                        // Check world model
                        vector<wire::PropertySet> objects = client.queryMAPObjects("/map");

                        // Save world model in map frame (needed in case multiple oredered objects are found)
                        map<string, pair<geometry_msgs::Point, string> > world_model_copy = getWorldModelObjects(objects);

                        // Check for ordered objects
                        checkOrderWithWorldModelAtShelf(world_model_copy, it_locations->second);

                    }

                    //! Reset head
                    moveHead(0.0, 0.0);

                }
            }

        }

        // Check if all orders are finished
        bool done = true;
        map<int, pair<string, string> >::iterator it_orders = order_map_.begin();
        for (; it_orders != order_map_.end(); ++it_orders) {
            if (it_orders->second.first != "Finished")
            {
                done = false;
            }
        }

        // Check if the Amigo is done
        if (done || n_loops > 3)
        {
            state = 3;
        }

        ++n_loops;
        follow_rate.sleep();
    }

    while (state == 3) {

        // Then go back to order location
        if (location_map_.find("ordering_location") != location_map_.end()) {

            ROS_DEBUG("Ordering location is (%f,%f)",
                      location_map_["ordering_location"].getOrigin().x(), location_map_["ordering_location"].getOrigin().x());

            if (!moveTowardsPositionMap(location_map_["ordering_location"])) {
                ROS_WARN("Could not go back to ordering location");
                amigoSpeak("I can not find my way to the ordering location yet", false);
                amigoRotate(3.0);
                if (!moveTowardsPositionMap(location_map_["ordering_location"])) {
                    ROS_WARN("I still can not reach the ordering location");
                    amigoSpeak("I can not reach the ordering location");

                }
            }
            else {
                ROS_INFO("Reached ordering location");
            }
        } else {
            ROS_WARN("No ordering location stored");
        }

        follow_rate.sleep();
        state = 4;

    }

    amigoSpeak("I finished all my tasks so I am done with the challenge!");

    //! When node is shut down, cancel goal by sending zero goal
    pbl::Matrix cov(3,3);
    cov.zeros();
    pbl::PDF pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
    moveTowardsPosition(pos, 0);

    delete planner_;
    delete listener;
    delete head_ref_ac_;
    delete left_arm_ac_;
    delete right_arm_ac_;
    delete reasoner_client;
    delete gripper_left_ac_;
    delete gripper_right_ac_;

    return 0;
}
