#include "wire_interface/Client.h"

#include <ros/ros.h>
#include <tue_move_base_msgs/MoveBaseAction.h>

// Action client
#include <actionlib/client/simple_action_client.h>

#include <pein_msgs/LearnAction.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <tf/transform_datatypes.h>
#include "problib/conversions.h"

#include <tf/transform_listener.h>

#include "challenge_follow_me/follow_me_carrot_planner.h"
#include <amigo_msgs/head_ref.h>

using namespace std;


//! Settings
const int TIME_OUT_OPERATOR_LOST = 10;          // Time interval without updates after which operator is considered to be lost
const double DISTANCE_OPERATOR = 2.0;           // Distance AMIGO keeps towards operator
const double WAIT_TIME_OPERATOR_MAX = 10.0;     // Maximum waiting time for operator to return
const string NAVIGATION_FRAME = "/base_link";   // Frame in which navigation goals are given IF NOT BASE LINK, UPDATE PATH IN moveTowardsPosition()
const int N_MODELS = 2;                         // Number of models used for recognition of the operator
const double TIME_OUT_LEARN_FACE = 90;          // Time out on learning of the faces
const double FOLLOW_RATE = 10;                   // Rate at which the move base goal is updated
double FIND_RATE = 1;                           // Rate check for operator at start of the challenge
double RESOLUTION_PATH = 0.1;                   // Resolution of the move base path

// NOTE: At this stage recognition is never performed, hence number of models can be small


//! Globals
CarrotPlanner* planner_;
double t_no_meas_ = 0;                                                            // Bookkeeping: determine how long operator is not observed
double t_last_check_ = 0;                                                         // Bookkeeping: last time operator position was checked
double last_var_operator_pos_ = -1;                                               // Bookkeeping: last variance in x-position operator
bool itp2 = false;                                                                // Bookkeeping: at elevator yes or no
bool itp3 = false;                                                                // Bookkeeping: passed elevator yes or no
actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* move_base_ac_; // Communication: Move base action client
actionlib::SimpleActionClient<pein_msgs::LearnAction>* learn_face_ac_;            // Communication: Learn face action client
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak
ros::ServiceClient reset_wire_client_;                                            // Communication: Client that enables reseting WIRE


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

    //! It is allowed to call the operator once per section (points for the section will be lost)
    if (lost) {
        amigoSpeak("I have lost my operator, can you please stand in front of me");
    }

    //! Give the operator some time to move to the robot
    ros::Duration wait_for_operator(4.0);
    wait_for_operator.sleep();

    //! See if the a person stands in front of the robot
    double t_start = ros::Time::now().toSec();
    ros::Duration dt(1.0);
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

                    //! If operator, set name in world model
                    if (pos_gauss.getMean()(0) < 2.0 && pos_gauss.getMean()(1) > -0.75 && pos_gauss.getMean()(1) < 0.75) {

                        amigoSpeak("I found my operator");

                        //! Reset
                        last_var_operator_pos_ = -1;
                        t_last_check_ = ros::Time::now().toSec();

                        //! Evidence
                        wire::Evidence ev(ros::Time::now().toSec());

                        //! Set the position
                        ev.addProperty("position", pos_gauss, NAVIGATION_FRAME);

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

                        //! Assert evidence to WIRE
                        client.assertEvidence(ev);

                        no_operator_found = false;
                        break;
                    }
                }

            }

        }

        dt.sleep();
    }
    
    ros::Duration safety_delta(1.0);
    safety_delta.sleep();
    
    
    //! Reset
    last_var_operator_pos_ = -1;
    t_last_check_ = ros::Time::now().toSec();

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
                    
                    ROS_INFO("Operator has variance %f, last variance is %f", cov(0,0), last_var_operator_pos_);


                    //! Check if operator position is updated (initially negative)
                    if (cov(0,0) < last_var_operator_pos_ || last_var_operator_pos_ < 0) {
                        last_var_operator_pos_ = cov(0,0);
                        t_no_meas_ = 0;
                        t_last_check_ = ros::Time::now().toSec();
                    } else {

                        //! Uncertainty increased: operator out of side
                        last_var_operator_pos_ = cov(0,0);
                        t_no_meas_ += (ros::Time::now().toSec() - t_last_check_);
                        ROS_INFO("%f [s] without position update operator: ", t_no_meas_);

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
 * @brief speechCallback
 * @param res
 */
void speechCallback(std_msgs::String res) {

    if (res.data == "pleaseenterthelevator") {
        ROS_INFO("Received command from operator: %s", res.data.c_str());
        itp2 = true;
    } else {
        ROS_WARN("Received unknown command \'%s\'", res.data.c_str());
    }
}


/**
 * @brief moveTowardsPosition Let AMIGO move from its current position towards the given position
 * @param pos target position
 * @param offset, in case the position represents the operator position AMIGO must keep distance
 */
void moveTowardsPosition(pbl::PDF& pos, double offset, tf::TransformListener& tf_listener) {


    pbl::Vector pos_exp = pos.getExpectedValue().getVector();
    /*
    tue_move_base_msgs::MoveBaseGoal move_base_goal;

    //! Plan a path from the current position
    geometry_msgs::PoseStamped start;
    start.header.frame_id = NAVIGATION_FRAME;
    start.pose.position.x = 0;
    start.pose.position.y = 0;
    start.pose.position.z = 0;

    tf::Quaternion q;
    q.setRPY(0, 0, 0);

    //! Set orientation
    start.pose.orientation.x = q.getX();
    start.pose.orientation.y = q.getY();
    start.pose.orientation.z = q.getZ();
    start.pose.orientation.w = q.getW();
    move_base_goal.path.push_back(start);

    */

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

    planner_->MoveToGoal(end_goal);

    /*
    //! Settings for interpolation of the desired path
    int nr_steps = (int)(max(abs((double)end_goal.pose.position.x), abs((double)end_goal.pose.position.y)) / RESOLUTION_PATH);
    double dx = end_goal.pose.position.x / nr_steps;
    double dy = end_goal.pose.position.y / nr_steps;

    //! Add interpolated way points (starting point is already added)
    for (int i = 1; i < nr_steps; ++i) {
        geometry_msgs::PoseStamped waypoint;
        waypoint = end_goal;
        waypoint.pose.position.x = i * dx;
        waypoint.pose.position.y = i * dy;
        move_base_goal.path.push_back(waypoint);
    }

    //! Add goal position
    move_base_goal.path.push_back(end_goal);

    //! Transform path to /map frame
    for(unsigned int i = 0; i < move_base_goal.path.size(); ++i) {
        geometry_msgs::PoseStamped waypoint_transformed;
        tf_listener.transformPose("/map", move_base_goal.path[i], waypoint_transformed);
        waypoint_transformed.pose.position.z = 0;
        move_base_goal.path[i] = waypoint_transformed;
    }

    //! Send goal to move base client
    move_base_ac_->sendGoal(move_base_goal);

*/

    ROS_INFO("Executive: Move base goal: (x,y,theta) = (%f,%f,%f)", end_goal.pose.position.x, end_goal.pose.position.y, theta);

}


void findAndEnterElevator(wire::Client& client, tf::TransformListener& tf_listener) {

    amigoSpeak("Okay, I will enter the elevator. Please step aside");
    ros::Duration delta(5.0);
    delta.sleep();

    // TODO: implement detection algorithm/service
    pbl::Matrix cov(3,3);
    cov.zeros();
    pbl::PDF el_pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);

    //! Enter elevator
    moveTowardsPosition(el_pos, 0.2, tf_listener);
    pbl::PDF el_pos_far = pbl::Gaussian(pbl::Vector3(1, 0, 0), cov);
    moveTowardsPosition(el_pos_far, 0.5, tf_listener);// perhaps drive for an extra meter to be sure the robot is inside the elevator?

    //! Turn at position (face entrance elevator
    pbl::PDF turn_pos = pbl::Gaussian(pbl::Vector3(-0.1, 0, 0), cov);
    moveTowardsPosition(turn_pos, 0, tf_listener);
    amigoSpeak("Operator, you can join me now");

    //! Find operator
    findOperator(client, false);


}



int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me_simple");
    ros::NodeHandle nh;
    
     ROS_INFO("Started Follow me");
    
    /// to publish goals to the head Kinect, necessary to init the head downwards, other goals are handled by executives
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
    

    //! Planner
    planner_ = new CarrotPlanner("follow_me_carrot_planner");
    ROS_INFO("Carrot planner instantiated");

    //! Query WIRE for objects
    wire::Client client;
    ROS_INFO("Wire client instantiated");

    //! Client that allows reseting WIRE
    reset_wire_client_ = nh.serviceClient<std_srvs::Empty>("/wire/reset");

    //! Subscribe to the speech recognition topic
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/speech_recognition_follow_me/output", 10, speechCallback);
    bool itp2 = false;

    //! Topic that makes AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/amigo_speak_up", 10);

    //! Action client for sending move base goals
    //move_base_ac_ = new actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>("move_base",true);
    //move_base_ac_->waitForServer();
    //ROS_INFO("Move base client connected to the move base server");

    //! Action client for learning faces
    learn_face_ac_ = new actionlib::SimpleActionClient<pein_msgs::LearnAction>("/face_learning/action_server",true);
    learn_face_ac_->waitForServer();
    ROS_INFO("Learn face client connected to the learn face server");

    //! Transform listener (localization required)
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(10));
    ROS_INFO("Transform listener ready");
    
    //! Always clear the world model
    std_srvs::Empty srv;
    if (reset_wire_client_.call(srv)) {
        ROS_INFO("Cleared world model");
    } else {
        ROS_ERROR("Failed to clear world model");
    }

    //! Administration
    pbl::PDF operator_pos;

    //! Create a model for identifying the operator later
    if (!memorizeOperator()) {

        ROS_ERROR("Learning operator failed: AMIGO will not be able to recognize the operator");
        findOperator(client);

    } else {

        //! Operator must be in the world model, remember position
        unsigned int n_tries = 0;
        ros::Rate find_rate(FIND_RATE);
        while(ros::ok()) {
            ros::spinOnce();

            //! Avoid too much delay due to some failure in perception
            if (n_tries > 10) {
                findOperator(client);
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

    //! Follow operator
    ros::Rate follow_rate(FOLLOW_RATE);
    while(ros::ok()) {
        ros::spinOnce();

        //! Get objects from the world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);

        //! Check if the robot arrived at itp two
        if (itp2) {

            //! Robot is asked to enter elevator and must leave when the operator leaves

            //! Find elevator
            findAndEnterElevator(client, tf_listener);

            itp2 = false;
            itp3 = true;

        } else if (itp3) {

            //! Operator will pass through a small crowd of people (4-5) and calls the robot from behind the group

            //! Association will fail, hence this can be skipped. Check for the (updated) operator position
            if (getPositionOperator(objects, operator_pos)) {

                //! Move towards operator
                moveTowardsPosition(operator_pos, DISTANCE_OPERATOR, tf_listener);


            } else {

                //! If the operator is lost, move around a crowd croud (wild guess)
                pbl::Matrix cov(3,3);
                cov.zeros();

                // Forward
                pbl::PDF pos = pbl::Gaussian(pbl::Vector3(1, 0, 0), cov);
                moveTowardsPosition(pos, 0, tf_listener);

                // Side
                pos = pbl::Gaussian(pbl::Vector3(0, 3, 0), cov);
                moveTowardsPosition(pos, 0, tf_listener);

                // Forward
                pos = pbl::Gaussian(pbl::Vector3(5, 0, 0), cov);
                moveTowardsPosition(pos, 0, tf_listener);

                // Back
                pos = pbl::Gaussian(pbl::Vector3(0, -3, 0), cov);
                moveTowardsPosition(pos, 0, tf_listener);

                // Wild guess for operator
                findOperator(client, false);

                itp3 = false;
            }



        } else {

            // Not at itp2/itp3, just follow operator

            //! Check for the (updated) operator position
            if (getPositionOperator(objects, operator_pos)) {

                //! Move towards operator
                moveTowardsPosition(operator_pos, DISTANCE_OPERATOR, tf_listener);


            } else {

                //! Lost operator
                findOperator(client);
            }
        }

        follow_rate.sleep();
    }

    return 0;
}
