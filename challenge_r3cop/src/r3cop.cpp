// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Messages
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <pein_msgs/LearnAction.h>

// Services
#include "perception_srvs/StartPerception.h"
#include <std_srvs/Empty.h>
#include "speech_interpreter/GetInfo.h"
#include "challenge_restaurant/SmachStates.h"

// Actions
#include <tue_move_base_msgs/MoveBaseAction.h>
#include <amigo_head_ref/HeadRefAction.h>

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
const int TIME_OUT_OPERATOR_LOST = 10;               // Time interval without updates after which operator is considered to be lost
const double DISTANCE_OPERATOR = 1.0;                // Distance AMIGO keeps towards operator
const double WAIT_TIME_OPERATOR_MAX = 10.0;          // Maximum waiting time for operator to return
const string NAVIGATION_FRAME = "/amigo/base_link";  // Frame in which navigation goals are given IF NOT BASE LINK, UPDATE PATH IN moveTowardsPosition()
const double FOLLOW_RATE = 20;                       // Rate at which the move base goal is updated
const double TYPICAL_OPERATOR_X = 1.0;               // Expected x-position operator, needed when looking for operator
const double TYPICAL_OPERATOR_Y = 0;                 // Expected y-position operator, needed when looking for operator
bool FIRST_OBSERVATION = false;

const double PI = 3.1415;


//! Globals
CarrotPlanner* planner_;
double t_no_meas_ = 0;                                                            // Bookkeeping: determine how long operator is not observed
double t_last_check_ = 0;                                                         // Bookkeeping: last time operator position was checked
double last_var_operator_pos_ = -1;                                               // Bookkeeping: last variance in x-position operator

// Actions
actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* move_base_ac_; // Communication: Move base action client

actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>* head_ac_; // Communication: head ref action client

// Publishers/subscribers
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak

// Services
ros::ServiceClient reset_wire_client_;                                            // Communication: Client that enables reseting WIRE
ros::ServiceClient speech_client_;                                                // Communication: Communication with the speech interpreter
ros::ServiceClient grab_machine_client_;                                          // Communication: Connection with (python) grab machinehead_ac_

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

void sendHeadGoal(double pan, double tilt) {
    amigo_head_ref::HeadRefGoal head_goal;
    head_goal.goal_type = amigo_head_ref::HeadRefGoal::PAN_TILT;
    head_goal.pan = pan;
    head_goal.tilt = tilt;

    ROS_INFO("Sending head goal: pan = %f, tilt = %f", pan, tilt);

    head_ac_->sendGoal(head_goal);
    head_ac_->waitForResult();
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
    if (lost) {

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
                        ROS_INFO("r3cop (findOperator): Position person is not a Gaussian");
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

            if (lost) amigoSpeak("I will continue following you");

            ros::Duration safety_delta(1.0);
            safety_delta.sleep();
            return;

        }

        dt.sleep();
    }

    amigoSpeak("I did not find my operator yet");

    return;

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




int main(int argc, char **argv) {
    ros::init(argc, argv, "r3cop_executive");
    ros::NodeHandle nh;

    ROS_INFO("Started r3cop demo");
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
    head_ac_ = new actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>("/head_ref_action", true);
    head_ac_->waitForServer();

    /// set the head to look down in front of AMIGO
    sendHeadGoal(0, 0.2);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Carrot planner
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double max_vel_lin = 0.5;
    double max_vel_rot = 0.4;
    double dist_wall = 0.8;
    planner_ = new CarrotPlanner("r3cop_carrot_planner", max_vel_lin, max_vel_rot, dist_wall);
    ROS_INFO("Carrot planner instantiated");


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Perception
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    //// Connect to grab maching
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    grab_machine_client_ = nh.serviceClient<challenge_restaurant::SmachStates>("/smach_states");

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

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Wait for the user to say continue
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    amigoSpeak("I will start following when you say continue");

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


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Start challenge: get infusion
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    amigoSpeak("Let's see if I can find your infusion.");

    // Look to the left, then look to the right
    for (unsigned int i=0; i < 2; ++i)
    {
        // Send head goal
        double pan = 0.7;
        if (i > 0) {
            pan = -0.7;
        }
        sendHeadGoal(pan, 0.4);

        // Switch on QR code detection
        perception_srvs::StartPerception pein_srv_qr;
        pein_srv_qr.request.modules.push_back("qr_detector");
        if (ppl_det_client.call(pein_srv_qr))
        {
            ROS_INFO("Switched on qr_detector");
        }

        // Give the QR detector some time to do detections
        ros::Duration(2.5).sleep();

        // Switch off qr detector (but turn on ppl detection)
        pein_srv_qr.request.modules.clear();
        pein_srv_qr.request.modules.push_back("ppl_detection");
        if (ppl_det_client.call(pein_srv_qr))
        {
            ROS_INFO("Switched off qr_detector");
        }

    }

    // Amigo should look up (again)
    sendHeadGoal(0, -0.2);

    amigoSpeak("I found your infusion.");

    // Call service to python grab machine
    challenge_restaurant::SmachStates ss_srv;
    ss_srv.request.state = "grasp";
    if (!grab_machine_client_.call(ss_srv)) {
        ROS_WARN("Service call to grab_machine failed");
    } else if (ss_srv.response.outcome != "Succeeded") {
        ROS_WARN("Service call returned %s", ss_srv.response.outcome.c_str());
        amigoSpeak("I am not able to get the infusion, I will follow you anyway");
    } else {
        ROS_INFO("Grab machine succeeded!");
        amigoSpeak("Let's go.");
    }

    bool first = true;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// START MAIN LOOP
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Rate follow_rate(FOLLOW_RATE);
    while(ros::ok()) {
        ros::spinOnce();

        //! Get objects from the world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);

        //! Check for the (updated) operator position
        if (getPositionOperator(objects, operator_pos)) {

            //! Move towards operator
            moveTowardsPosition(operator_pos, DISTANCE_OPERATOR);


        } else {

            //! Lost operator
            findOperator(client, !first);
            first = false;
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
