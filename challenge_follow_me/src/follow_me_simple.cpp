// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Messages
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <amigo_msgs/head_ref.h>
#include <pein_msgs/LearnAction.h>
#include <sensor_msgs/LaserScan.h>

// Services
#include "perception_srvs/StartPerception.h"
#include <std_srvs/Empty.h>
#include "speech_interpreter/GetInfo.h"

// Actions
#include <tue_move_base_msgs/MoveBaseAction.h>

// Action client
#include <actionlib/client/simple_action_client.h>

// WIRE
#include "wire_interface/Client.h"
#include "problib/conversions.h"

// Carrot planner
#include "tue_carrot_planner/carrot_planner.h"

// Speech recognition
#include "tue_pocketsphinx/Switch.h"

#include <map>

using namespace std;


//! Settings
const int TIME_OUT_OPERATOR_LOST = 10;          // Time interval without updates after which operator is considered to be lost
const double DISTANCE_OPERATOR = 1.0;           // Distance AMIGO keeps towards operator
const double WAIT_TIME_OPERATOR_MAX = 10.0;     // Maximum waiting time for operator to return
const string NAVIGATION_FRAME = "/base_link";   // Frame in which navigation goals are given IF NOT BASE LINK, UPDATE PATH IN moveTowardsPosition()
const int N_MODELS = 2;                         // Number of models used for recognition of the operator
const double TIME_OUT_LEARN_FACE = 25;          // Time out on learning of the faces
const double FOLLOW_RATE = 20;                  // Rate at which the move base goal is updated
double FIND_RATE = 1;                           // Rate check for operator at start of the challenge
const double T_LEAVE_ELEVATOR = 15.0;            // Time after which robot is assumed to be outside the elevator.
const double TYPICAL_OPERATOR_X = 1.0;          // Expected x-position operator, needed when looking for operator
const double TYPICAL_OPERATOR_Y = 0;            // Expected y-position operator, needed when looking for operator
const double MAX_ELEVATOR_WALL_DISTANCE = 2.0;  // Maximum distance of robot to wall in elevator (used to detect elevator)
const double ELEVATOR_INLIER_RATIO = 0.75;      // % of laser points that should at least be within bounds for elevator to be detected

const double PI = 3.1415;

// NOTE: At this stage recognition is never performed, hence number of models can be small
// TODO: Check/test if confimation is needed: please leave the elevator


//! Globals
CarrotPlanner* planner_;
double t_no_meas_ = 0;                                                            // Bookkeeping: determine how long operator is not observed
double t_last_check_ = 0;                                                         // Bookkeeping: last time operator position was checked
double last_var_operator_pos_ = -1;                                               // Bookkeeping: last variance in x-position operator
bool itp2_ = false;                                                               // Bookkeeping: at elevator yes or no
bool itp3_ = false;                                                               // Bookkeeping: passed elevator yes or no
bool new_laser_data_ = false;                                                     // Bookkeeping: new laser data or not
bool in_elevator_ = false;                                                        // Bookkeeping: Is robot in elevator?
pbl::Gaussian last_driving_dir_elevator_(3);                                      // Bookkeeping: remember driving direction when leaving elevator
sensor_msgs::LaserScan laser_scan_;                                               // Storage: most recent laser data

// Actions
actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* move_base_ac_; // Communication: Move base action client
actionlib::SimpleActionClient<pein_msgs::LearnAction>* learn_face_ac_;            // Communication: Learn face action client

// Publishers/subscribers
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak
ros::Subscriber sub_laser_;                                                       // Communication: Listen to laser data
ros::Publisher pub_in_elevator;                                                   // Communication: Publisher for debugging

// Services
ros::ServiceClient reset_wire_client_;                                            // Communication: Client that enables reseting WIRE
ros::ServiceClient speech_recognition_client_;                                    // Communication: Client for starting / stopping speech recognition
ros::ServiceClient speech_client_;                                                // Communication: Communication with the speech interpreter

/**
 * @brief amigoSpeak let AMIGO say a sentence
 * @param sentence
 */
void amigoSpeak(string sentence) {
    ROS_INFO("AMIGO: \'%s\'", sentence.c_str());
    std_msgs::String sentence_msgs;
    sentence_msgs.data = sentence;
    pub_speech_.publish(sentence_msgs);
}



/**
 * @brief findOperator, detects person in front of robot, empties WIRE and adds person as operator
 * @param client used to query from/assert to WIRE
 */
void findOperator(wire::Client& client, bool lost = true) {

    //! Region in front of robot in which operator is allowed to stand
    double distance_left_right_min = -0.75;
    double distance_left_right_max = 0.75;
    double distance_min = 0.25;
    double distance_max = 2.0;

    if (!lost)
    {
        distance_left_right_min = -1.1;
        distance_left_right_max = 1.1;
    }

    //! It is allowed to call the operator once per section (points for the section will be lost)
    if (lost && !in_elevator_) {

        //! If the robot is in the elevator, informing costs points
        amigoSpeak("I have lost my operator, can you please stand in front of me");

        //! Give the operator some time to move to the robot
        ros::Duration wait_for_operator(5.0);
        wait_for_operator.sleep();
    }

    //! Reset world model
    std_srvs::Empty srv;
    if (reset_wire_client_.call(srv)) {
        ROS_INFO("Cleared world model");
    } else {
        ROS_WARN("Failed to clear world model");
    }

    //! Always some time before operator is there
    //ros::Duration waiting_time(1.0);
    //waiting_time.sleep();

    //! Vector with candidate operators
    vector<pbl::Gaussian> vector_possible_operators;

    //! See if the a person stands in front of the robot
    double t_start = ros::Time::now().toSec();
    ros::Duration dt(0.5);
    bool no_operator_found = true;
    while (ros::Time::now().toSec() - t_start < WAIT_TIME_OPERATOR_MAX && no_operator_found) {

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

                    //! Get position of potential operator
                    pbl::PDF pos = prop_pos.getValue();
                    pbl::Gaussian pos_gauss(3);
                    if (pos.type() == pbl::PDF::GAUSSIAN) {
                        pos_gauss = pbl::toGaussian(pos);

                    } else {
                        ROS_INFO("follow_me_simple (findOperator): Position person is not a Gaussian");
                    }

                    //! Check if the person stands in front of the robot
                    if (pos_gauss.getMean()(0) < distance_max &&
                            pos_gauss.getMean()(0) > distance_min &&
                            pos_gauss.getMean()(1) > distance_left_right_min &&
                            pos_gauss.getMean()(1) < distance_left_right_max) {
                        vector_possible_operators.push_back(pos_gauss);
                        ROS_INFO("Found candidate operator at (x,y) = (%f,%f)", pos_gauss.getMean()(0), pos_gauss.getMean()(1));

                    }
                }

            }

        }

        //! Found at least one candidate operator
        if (!vector_possible_operators.empty()) {

            //! Position candidate operator
            pbl::Gaussian pos_operator(3);

            //! Find person that has smallest Eucledian distance to robot
            if (vector_possible_operators.size() > 1) {

                double dist_min = 0.0;
                int i_best = 0;

                for (unsigned int i = 0; i < vector_possible_operators.size(); ++i)
                {
                    double dx = TYPICAL_OPERATOR_X - vector_possible_operators[i].getMean()(0);
                    double dy = TYPICAL_OPERATOR_Y - vector_possible_operators[i].getMean()(1);
                    double dist = sqrt(dx*dx+dy*dy);
                    if (i == 0 || dist < dist_min)
                    {
                        dist_min = dist;
                        i_best = i;
                    }
                }

                pos_operator = vector_possible_operators[i_best];

            }
            //! In case there is only one candidate operator this person is the operator
            else {
                pos_operator = vector_possible_operators[0];
            }

            ROS_INFO("Found operator at (x,y) = (%f,%f)", pos_operator.getMean()(0), pos_operator.getMean()(1));

            //! Reset
            last_var_operator_pos_ = -1;
            t_last_check_ = ros::Time::now().toSec();

            //! Evidence
            wire::Evidence ev(t_last_check_);

            //! Set the position
            ev.addProperty("position", pos_operator, NAVIGATION_FRAME);

            //! Name must be operator
            pbl::PMF name_pmf;
            name_pmf.setProbability("operator", 1.0);
            ev.addProperty("name", name_pmf);

            //! Reset the world model
            std_srvs::Empty srv;
            if (reset_wire_client_.call(srv)) {
                ROS_INFO("Cleared world model");
            } else {
                ROS_ERROR("Failed to clear world model");
            }

            //! Assert evidence to WIRE (multiple times to be sure)
            for (unsigned int dummy = 0; dummy < 5; ++dummy)
            {
                client.assertEvidence(ev);
            }

            no_operator_found = false;

            if (!lost) amigoSpeak("I found my operator.");
            else amigoSpeak("I will continue following you");

            ros::Duration safety_delta(1.0);
            safety_delta.sleep();
            return;

        }

        dt.sleep();
    }

    amigoSpeak("I did not find my operator yet");

    return;

}





bool detectCrowd(vector<wire::PropertySet>& objects) {

    // Loop over world objects, if at least one person close to operator: crowd detected. Also determine width crowd.

    // Can be based on getPositionOperator()

    return true;
}



/**
* @brief getPositionOperator
* @param objects input objects received from WIRE
* @param pos position of the operator (output)
* @return bool indicating whether or not the operator was found
*/
bool getPositionOperator(vector<wire::PropertySet>& objects, pbl::PDF& pos) {

    //! Iterate over all world model objects
    for(vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        wire::PropertySet& obj = *it_obj;
        const wire::Property& prop_name = obj.getProperty("name");
        if (prop_name.isValid()) {

            //! Check if the object represents the operator
            if (prop_name.getValue().getExpectedValue().toString() == "operator") {
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid()) {

                    //! Get the operator's position
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

                    ROS_DEBUG("Operator has variance %f, last variance is %f", cov(0,0), last_var_operator_pos_);


                    //! Check if operator position is updated (initially negative)
                    if (cov(0,0) < last_var_operator_pos_ || last_var_operator_pos_ < 0) {
                        last_var_operator_pos_ = cov(0,0);
                        t_no_meas_ = 0;
                        t_last_check_ = ros::Time::now().toSec();
                    } else {

                        //! Uncertainty increased: operator out of side
                        last_var_operator_pos_ = cov(0,0);
                        t_no_meas_ += (ros::Time::now().toSec() - t_last_check_);
                        if (t_no_meas_ > 1) ROS_INFO("%f [s] without position update operator: ", t_no_meas_);

                        //! Position uncertainty increased too long: operator lost
                        if (t_no_meas_ > TIME_OUT_OPERATOR_LOST) {
                            ROS_INFO("I lost my operator");
                            return false;
                        }
                    }

                    t_last_check_ = ros::Time::now().toSec();

                    return true;

                } else {
                    ROS_WARN("Found an operator without valid position attribute");
                }
            }
        }
    }

    //! If no operator was found, return false
    return false;
}

/**
 * @brief Learn a model with name operator for the person standing in front of the robot
 * @return boolean indicating success of the learning action
 */
bool memorizeOperator() {

    //! Ask operator to look at AMIGO
    amigoSpeak("Please stand at one meter in front of me and look at me");

    //! Send learn face goal to the action server
    pein_msgs::LearnGoal goal;
    goal.module = "face_learning";
    goal.n_models = N_MODELS;
    goal.model_name = "operator";
    goal.publish_while_learning = true;
    goal.view = "front";

    if (learn_face_ac_->isServerConnected()) {
        learn_face_ac_->sendGoal(goal);

        //! Wait for the action to return
        if (learn_face_ac_->waitForResult(ros::Duration(TIME_OUT_LEARN_FACE))) {
            actionlib::SimpleClientGoalState state = learn_face_ac_->getState();
            ROS_INFO("Learn operator action finished: %s", state.toString().c_str());
            amigoSpeak("Thank you");
        }
        else  {
            ROS_WARN("Learn operator action did not finish before the time out.");
            return false;
        }

    } else {
        ROS_WARN("Not connected with the learn operator action server: no goal send");
        return false;
    }

    return true;

}

/**
 * @brief moveTowardsPosition Let AMIGO move from its current position towards the given position
 * @param pos target position
 * @param offset, in case the position represents the operator position AMIGO must keep distance
 */
void moveTowardsPosition(pbl::PDF& pos, double offset) {


    pbl::Vector pos_exp = pos.getExpectedValue().getVector();

    //! End point of the path is the given position
    geometry_msgs::PoseStamped end_goal;
    end_goal.header.frame_id = NAVIGATION_FRAME;
    double theta = atan2(pos_exp(1), pos_exp(0));
    tf::Quaternion q;
    q.setRPY(0, 0, theta);

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
        ROS_INFO("No operator position update: robot will not move");
    }

}


bool startSpeechRecognition() {
    std::string knowledge_path = ros::package::getPath("tue_knowledge");
    if (knowledge_path == "") {
        return false;
    }

    std::string follow_me_speech_path = knowledge_path + "/speech_recognition/follow_me/";

    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::START;
    req.hidden_markov_model = "/usr/share/pocketsphinx/model/hmm/wsj1";
    req.dictionary = follow_me_speech_path + "follow_me.dic";
    req.language_model = follow_me_speech_path + "follow_me.lm";

    tue_pocketsphinx::Switch::Response resp;
    if (speech_recognition_client_.call(req, resp)) {
        if (resp.error_msg == "") {
            ROS_INFO("Switched on speech recognition");
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
 * @brief speechCallback
 * @param res
 */
void speechCallback(std_msgs::String res) {

    // TODO: If this becomes problematic, add distance to operator check

    //amigoSpeak(res.data);
    if (!itp2_ && !itp3_ && res.data == "amigoleave" && in_elevator_) { //res.data.find("elevator") != std::string::npos) {
        ROS_WARN("Received command: %s", res.data.c_str());
        itp2_ = true;
    } else {
        ROS_DEBUG("Received unknown command \'%s\' or already leaving the elevator", res.data.c_str());
    }

    // always immediately start listening again
    startSpeechRecognition();
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg){

    //! Store data
    new_laser_data_ = true;
    laser_scan_ = *laser_scan_msg;

    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    //   ELEVATOR DETECTOR
    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    vector<int> num_total_points(3, 0);
    vector<int> num_points_in_bounds(3, 0);

    double angle = laser_scan_msg->angle_min;
    for(unsigned int i = 0; i < laser_scan_msg->ranges.size(); ++i) {
        double range = laser_scan_msg->ranges[i];

        // check left of robot
        if (angle > -0.5 * PI && angle < -0.25 * PI) {
            num_total_points[0]++;
            if (range < MAX_ELEVATOR_WALL_DISTANCE) {
                num_points_in_bounds[0]++;
            }

            // check in front of robot
        } else if (angle > -0.25 * PI && angle < 0.25 * PI) {
            num_total_points[1]++;
            if (range < MAX_ELEVATOR_WALL_DISTANCE) {
                num_points_in_bounds[1]++;
            }

            // check right of robot
        } else if (angle > 0.25 * PI && angle < 0.5 * PI) {
            num_total_points[2]++;
            if (range < MAX_ELEVATOR_WALL_DISTANCE) {
                num_points_in_bounds[2]++;
            }
        }

        angle += laser_scan_msg->angle_increment;
    }

    // check if all parts have enough 'inliers'
    in_elevator_ = true;
    for(unsigned int i = 0; i < num_total_points.size(); ++i) {
        if ((double)num_points_in_bounds[i] / num_total_points[i] < ELEVATOR_INLIER_RATIO) {
            in_elevator_ = false;
            break;
        }
    }

    // publish for debugging purposes
    std_msgs::Bool msg_in_elevator;
    msg_in_elevator.data = in_elevator_;
    pub_in_elevator.publish(msg_in_elevator);
}



bool leftElevator(pbl::Gaussian& pos)
{

    //! Only proceed if new laser data is available
    if (!new_laser_data_)
    {
        ROS_WARN("Trying to leave the elevator but no new laser data available");
        return false;
    }

    //! Administration
    new_laser_data_ = false;
    pbl::Matrix cov(3,3);
    cov.zeros();
    pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);


    //! Settings
    double min_dist_exit = 1.5;
    double wdt_robot = 0.75;
    unsigned int min_n_beams = 2.0*atan2(wdt_robot/2, min_dist_exit) / laser_scan_.angle_increment;


    //! Administration
    unsigned int i = 0;
    map<unsigned int, int> beam_n_beams_map;

    //! Get candidate exits
    ROS_DEBUG("Start looking for an elevator exit...");
    double distance_left = 0, distance_right = 0;
    while (i < laser_scan_.ranges.size()-1)
    {
        unsigned int i_first = i;
        unsigned int i_current = i;
        double min_distance_segment = 0.0;

        //! Determine size of segment to possible exit
        while (laser_scan_.ranges[i_current] > min_dist_exit)
        {
            if (i_current == i_first || laser_scan_.ranges[i_current] < min_distance_segment)
            {
                min_distance_segment = laser_scan_.ranges[i_current];
            }

            //! Check distance on left/right of AMIGO
            if (std::fabs(laser_scan_.angle_min + i_current*laser_scan_.angle_increment + PI/2) < laser_scan_.angle_increment) {
                distance_left = laser_scan_.ranges[i_current];
            } else if (std::fabs(laser_scan_.angle_min + i_current*laser_scan_.angle_increment - PI/2) < laser_scan_.angle_increment) {
                distance_right = laser_scan_.ranges[i_current];
            }

            ++i_current;
        }

        //! Store most promosing segment: allows for providing feedback on various options later
        if (i_current-i_first > min_n_beams)
        {
            beam_n_beams_map[(i_first+i_current)/2] = i_current-i_first;
            ROS_DEBUG("Found possible exit, %u beams", i_current-i_first);
        }

        i = ++i_current;

    }

    //! Check if AMIGO already left the elevator
    if (distance_left > 2.5 && distance_right > 2.5) {
        ROS_WARN("AMIGO is outside the elevator: distances left and right are %f and %f", distance_left, distance_right);
        
        const double time_wait = 2.0;
        const unsigned int freq = 20;
        unsigned int N_SLEEPS_TOTAL = time_wait*freq;
        ros::Duration pause(1.0/(double)freq);
        unsigned int n_sleeps = 0;
        while (n_sleeps < N_SLEEPS_TOTAL) {
            moveTowardsPosition(last_driving_dir_elevator_, 0);
            pause.sleep();
            ++n_sleeps;
        }
        ;
        return true;
    }


    //! Determine most likely candidate
    ROS_DEBUG("Finished iterating over laser data");

    // Limited distance to keep the velocity low
    double distance_drive = 0.6; // TODO: Must be larger?
    if (beam_n_beams_map.size() == 1)
    {

        double angle_exit = laser_scan_.angle_min + beam_n_beams_map.begin()->first * laser_scan_.angle_increment;
        ROS_INFO(" angle towards exit is %f, beam %u", angle_exit, beam_n_beams_map.begin()->first);

        pos = pbl::Gaussian(pbl::Vector3(cos(angle_exit)*distance_drive, sin(angle_exit)*distance_drive, 0), cov);
        ROS_INFO("Relative angle to exit is %f, corresponding number of beams is %d", angle_exit, beam_n_beams_map.begin()->second);
    } else if (beam_n_beams_map.size() > 1)
    {

        ROS_INFO("%zu candidate exits", beam_n_beams_map.size());

        //! Iterate over map and get segment with largest number of beams
        int n_beams_max = 0;
        map<unsigned int, int>::const_iterator it_best = beam_n_beams_map.begin();
        map<unsigned int, int>::const_iterator it = beam_n_beams_map.begin();
        for (; it != beam_n_beams_map.end(); ++it)
        {
            if (it->second > n_beams_max)
            {
                it_best = it;
                n_beams_max = it->second;
            }
        }

        ROS_DEBUG("\tfound most probable exit");
        double angle_exit = laser_scan_.angle_min + it_best->first * laser_scan_.angle_increment;
        ROS_INFO("\tangle towards exit is %f", angle_exit);

        //! Check if the robot left the elevator
        //if (angle_exit > PI)
        //{
        //    return true;
        //}

        pos = pbl::Gaussian(pbl::Vector3(cos(angle_exit)*distance_drive, sin(angle_exit)*distance_drive, 0), cov);
        ROS_INFO("\tRelative angle to exit is %f", angle_exit);

    }
    else
    {
        ROS_INFO("No candidate exits found, just turn");
        pos = pbl::Gaussian(pbl::Vector3(-1, 0, 0), cov);
        moveTowardsPosition(pos, 1);
        return false;

    }

    //! Let robot drive the right direction
    last_driving_dir_elevator_ = pos;
    moveTowardsPosition(pos, 0);
    return false;

}




int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me_simple");
    ros::NodeHandle nh;

    ROS_INFO("Started Follow me");
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////  Text-to-speech
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    pub_speech_ = nh.advertise<std_msgs::String>("/text_to_speech/input", 10);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Speech interpreter
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    speech_client_ = nh.serviceClient<speech_interpreter::GetInfo>("interpreter/get_info_user");

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Head ref
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Publisher head_ref_pub = nh.advertise<amigo_msgs::head_ref>("/head_controller/set_Head", 1);

    /// set the head to look down in front of AMIGO
    ros::Rate poll_rate(100);
    while (head_ref_pub.getNumSubscribers() == 0) {
        ROS_INFO_THROTTLE(1, "Waiting to connect to head ref topic...");
        poll_rate.sleep();
    }
    ROS_INFO("Sending head ref goal");
    amigo_msgs::head_ref goal;
    goal.head_pan = 0.0;
    goal.head_tilt = -0.2;
    head_ref_pub.publish(goal);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Carrot planner
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    planner_ = new CarrotPlanner("follow_me_carrot_planner");
    ROS_INFO("Carrot planner instantiated");

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Laser data
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    sub_laser_ = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 10, laserCallback);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Speech-to-text
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // connect to pocketsphinx
    speech_recognition_client_ = nh.serviceClient<tue_pocketsphinx::Switch>("/pocketsphinx/switch");
    speech_recognition_client_.waitForExistence();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Face learning and perception
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    learn_face_ac_ = new actionlib::SimpleActionClient<pein_msgs::LearnAction>("/face_learning/action_server",true);
    learn_face_ac_->waitForServer();
    ROS_INFO("Learn face client connected to the learn face server");
    ros::ServiceClient ppl_det_client = nh.serviceClient<perception_srvs::StartPerception>("/start_perception");
    perception_srvs::StartPerception pein_srv;
    pein_srv.request.modules.push_back("ppl_detection");
    if (ppl_det_client.call(pein_srv))
    {
        ROS_INFO("Switched on laser_ppl_detection");
    }
    else
    {
        ROS_ERROR("Failed to switch on perception");
        ros::Duration wait(1.0);
        wait.sleep();
        if (!ppl_det_client.call(pein_srv))
        {
            ROS_ERROR("No ppl detection possible, end of challenge");
            return 1;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// WIRE
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    wire::Client client;
    ROS_INFO("Wire client instantiated");
    reset_wire_client_ = nh.serviceClient<std_srvs::Empty>("/wire/reset");
    std_srvs::Empty wire_srv;
    if (reset_wire_client_.call(wire_srv)) {
        ROS_INFO("Cleared world model");
    } else {
        ROS_ERROR("Failed to clear world model");
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Administration variables
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pbl::PDF operator_pos;
    itp2_ = false;
    bool itp2_new = true;
    itp3_ = false;
    unsigned int n_checks_left_elevator = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Debugging
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    pub_in_elevator = nh.advertise<std_msgs::Bool>("/is_in_elevator", 10);

    // Wait for continue to start the challenge
    speech_client_.waitForExistence(ros::Duration(5.0));
    amigoSpeak("I will start when you say continue");

    double t1 = ros::Time::now().toSec();
    speech_interpreter::GetInfo srv_test;
    srv_test.request.n_tries = 6;
    srv_test.request.time_out = 60.0;
    srv_test.request.type = "continue_confirm";
    if (speech_client_.call(srv_test) && srv_test.response.answer == "continue") {
        ROS_INFO("Received true");
    } else {
        ROS_WARN("I heard %s - start anyway", srv_test.response.answer.c_str());
    }

    ROS_INFO("Waited %f [s]", ros::Time::now().toSec()-t1);

    // Non-blocking
    amigoSpeak("I will start the challenge");

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Start challenge: find and learn the operator
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (!memorizeOperator()) {

        ROS_ERROR("Learning operator failed: AMIGO will not be able to recognize the operator");
        //ros::Duration wait_for_operator(3.0);
        //wait_for_operator.sleep();
        findOperator(client, false);

    } else {

        //! Operator must be in the world model, remember position
        unsigned int n_tries = 0;
        ros::Rate find_rate(FIND_RATE);
        while(ros::ok()) {
            ros::spinOnce();

            //! Avoid too much delay due to some failure in perception
            if (n_tries > 5) {
                findOperator(client, false);
                ROS_ERROR("Learning OK but no operator in world model, person in front of the robot is assumed to be the operator");
                break;
            }

            //! Get objects in estimated world state
            vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);
            t_last_check_ = ros::Time::now().toSec();

            //! Start challenge once the operator is found
            if (getPositionOperator(objects, operator_pos)) {
                break;
            }

            ROS_INFO("No operator found, waiting for operator...");
            ++n_tries;
            find_rate.sleep();
        }
    }

    ROS_INFO("Found operator with position %s in frame \'%s\'", operator_pos.toString().c_str(), NAVIGATION_FRAME.c_str());
    amigoSpeak("I will now start following you");

    // Start speech recognition
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 10, speechCallback);
    startSpeechRecognition();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// START MAIN LOOP
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Rate follow_rate(FOLLOW_RATE);
    while(ros::ok()) {
        ros::spinOnce();

        //! Get objects from the world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);

        ROS_DEBUG("itp2_ is %s", itp2_?"true":"false");

        //! Check if the robot arrived at itp two
        if (itp2_) {

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // * * * * * * * * * * * * * * * * * * * * * * * * * * * ITP 2 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            pbl::Matrix cov(3,3);
            cov.zeros();
            pbl::Gaussian pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);

            // First time here, rotate towards exit
            if (itp2_new) {

                //! Robot is asked to leave the elevator
                amigoSpeak("I will leave the elevator now. Can you wait until I call you?");

                sub_laser_ = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 10, laserCallback);
                ROS_INFO("Subscribed to laser data");

                //! Rotate for 180 deg (in steps since only small angles allowed)
                pos = pbl::Gaussian(pbl::Vector3(-1, 0, 0), cov);
                const double time_rotation = 7.0;
                const unsigned int freq = 20;
                unsigned int N_SLEEPS_TOTAL = time_rotation*freq;
                ros::Duration pause(1.0/(double)freq);
                unsigned int n_sleeps = 0;

                while (n_sleeps < N_SLEEPS_TOTAL) {
                    moveTowardsPosition(pos, 2);
                    pause.sleep();
                    ++n_sleeps;
                    //ROS_INFO("n_sleeps = %u, N_SLEEPS_TOTAL = %u, freq = %u", n_sleeps, N_SLEEPS_TOTAL, freq);
                }

                //! Stand still
                pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
                moveTowardsPosition(pos, 0);

                itp2_new = false;

            }

            //! Always maker sure laser data is available
            if (sub_laser_.getTopic().empty())
            {
                ROS_WARN("ITP2: Subscriber not registered, subscribing now.");
                sub_laser_ = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 10, laserCallback);
            }

            //! If left elevator (function does driving!)
            if (leftElevator(pos) || n_checks_left_elevator > T_LEAVE_ELEVATOR*FOLLOW_RATE)
            {

                //! Rotate for 90 deg (in steps since only small angles allowed)
                pos = pbl::Gaussian(pbl::Vector3(-2, 0, 0), cov);
                const double time_rotation = 7.0;
                const unsigned int freq = 20;
                unsigned int N_SLEEPS_TOTAL = time_rotation*freq;
                ros::Duration pause(1.0/(double)freq);
                unsigned int n_sleeps = 0;

                while (n_sleeps < N_SLEEPS_TOTAL) {
                    moveTowardsPosition(pos, 2);
                    pause.sleep();
                    ++n_sleeps;
                }

                //! Clear world model (operator lost to avoid incorrect association in base link)
                if (reset_wire_client_.call(wire_srv)) {
                    ROS_INFO("Cleared world model");
                } else {
                    ROS_ERROR("Failed to clear world model");
                }

                //! Ask operator to come
                amigoSpeak("I left the elevator, you can leave the elevator now");

                //! Stand still and find operator
                pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
                moveTowardsPosition(pos, 0);

                //! Find the operator (blocks until the operator is found)
                findOperator(client, false);

                //! Next state: not implemented, go back to normal following mode
                itp2_ = false;
                itp3_ = false;
            }
            else
            {
                ROS_DEBUG("n_checks_left_elevator = %u/%f", n_checks_left_elevator, T_LEAVE_ELEVATOR*FOLLOW_RATE);
                ++n_checks_left_elevator;
            }


        } else if (itp3_) {

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // * * * * * * * * * * * * * * * * * * * * * * * * * * * ITP 3 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            //! Operator will pass through a small crowd of people (4-5) and calls the robot from behind the group

            /*
            if (detectCrowd(objects)) {

                ////
                // TODO: Detecting crowd is feasible, but how should the robot drive around the crowd?
                //       Option 1 is assuming that driving a fixed length, e.g., 4 [m] is enough to
                //        pass the crowd (first move sidewards, then 4 [m] forward, then turn and call
                //        find(operator(client, false).
                //       Option 2 is using some form of feedback, e.g., by entering a separate mode that
                //        lets the robot drive as long as at least one (two?) person is on its side.
                //        Potential risk is that this can be the operator already, furthermore, this
                //        will require a new bool itp3_crowd_ and some hacks? Maybe the potential risk
                //        isn't even a real problem: the operator will call the robot, hence the mic
                //        and or face detection can be of help here.
                ////

                ROS_INFO("Found crowd!");

                // TODO: try to drive around the crowd

            } else {

                //! Just follow
                if (getPositionOperator(objects, operator_pos)) {

                    //! Move towards operator
                    moveTowardsPosition(operator_pos, DISTANCE_OPERATOR);


                } else {

                    //! Lost operator
                    findOperator(client);

                }

            }*/

            /*

            //! Still the operator position is desired
            if (getPositionOperator(objects, operator_pos)) {

                //! Move towards operator
                moveTowardsPosition(operator_pos, DISTANCE_OPERATOR);


                //! If distance towards operator is smaller than this value, crowd is assumed
                pbl::Vector pos_exp = operator_pos.getExpectedValue().getVector();
                if (pos_exp[0] < 0.9)
                {

                    // Drive fixed path and hope for the best
                    pbl::Matrix cov(3,3);
                    cov.zeros();

                    // Sidewards
                    pbl::PDF pos = pbl::Gaussian(pbl::Vector3(0, 2, 0), cov);
                    moveTowardsPosition(pos, 0);
                    ros::Duration delta1(3.0);
                    delta1.sleep();

                    // Freeze
                    pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
                    moveTowardsPosition(pos, 0);

                    // Forward
                    pos = pbl::Gaussian(pbl::Vector3(2, 0, 0), cov);
                    moveTowardsPosition(pos, 0);
                    ros::Duration delta2(4.0);
                    delta2.sleep();

                    // Freeze
                    pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
                    moveTowardsPosition(pos, 0);

                    // Sidewards (back)
                    pos = pbl::Gaussian(pbl::Vector3(0, -2, 0), cov);
                    moveTowardsPosition(pos, 0);
                    ros::Duration delta3(3.0);
                    delta3.sleep();

                    // Freeze
                    pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
                    moveTowardsPosition(pos, 0);

                    // Wild guess for operator
                    findOperator(client, false);

                    itp3_ = false;

                }


            }*/


        } else {

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // * * * * * * * * * * * * * * * * * * * * * * * * * * * ITP 1 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Not at itp2/itp3, just follow operator

            //! Check for the (updated) operator position
            if (getPositionOperator(objects, operator_pos)) {

                //! Move towards operator
                moveTowardsPosition(operator_pos, DISTANCE_OPERATOR);


            } else {

                //! Lost operator
                findOperator(client);
            }
        }

        follow_rate.sleep();
    }

    //! When node is shut down, cancel goal by sending zero
    pbl::Matrix cov(3,3);
    cov.zeros();
    pbl::PDF pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
    moveTowardsPosition(pos, 0);

    return 0;
}
