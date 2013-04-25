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

#include "challenge_restaurant/restaurant_carrot_planner.h"
#include <amigo_msgs/head_ref.h>

using namespace std;


//! Settings
const int TIME_OUT_GUIDE_LOST = 5;          // Time interval without updates after which operator is considered to be lost
const double DISTANCE_GUIDE = 2.0;           // Distance AMIGO keeps towards guide
const double WAIT_TIME_GUIDE_MAX = 5.0;     // Maximum waiting time for guide to return
const string NAVIGATION_FRAME = "/base_link";   // Frame in which navigation goals are given IF NOT BASE LINK, UPDATE PATH IN moveTowardsPosition()
const double FOLLOW_RATE = 10;                   // Rate at which the move base goal is updated
double FIND_RATE = 5;                           // Rate check for guide at start of the challenge

// NOTE: At this stage recognition is never performed, hence number of models can be small


//! Globals
CarrotPlanner* planner_;
double t_no_meas_ = 0;                                                            // Bookkeeping: determine how long guide is not observed
double t_last_check_ = 0;                                                         // Bookkeeping: last time guide position was checked
double last_var_guide_pos_ = -1;                                               // Bookkeeping: last variance in x-position guide
actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* move_base_ac_; // Communication: Move base action client
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
 * @brief findGuide, detects person in front of robot, empties WIRE and adds person as guide
 * @param client used to query from/assert to WIRE
 */
void findGuide(wire::Client& client, bool lost = true) {

    //! It is allowed to call the guide once per section (points for the section will be lost)
    if (lost) {
        amigoSpeak("I have lost my guide, can you please stand in front of me");
    }

    //! Give the guide some time to move to the robot
    ros::Duration wait_for_guide(6.0);
    wait_for_guide.sleep();

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

                    //! If guide, set name in world model
                    if (pos_gauss.getMean()(0) < 2.0 && pos_gauss.getMean()(1) > -0.75 && pos_gauss.getMean()(1) < 0.75) {

                        amigoSpeak("I found my guide");

                        //! Reset
                        last_var_guide_pos_ = -1;
                        t_last_check_ = ros::Time::now().toSec();

                        //! Evidence
                        wire::Evidence ev(ros::Time::now().toSec());

                        //! Set the position
                        ev.addProperty("position", pos_gauss, NAVIGATION_FRAME);

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

                        //! Assert evidence to WIRE
                        client.assertEvidence(ev);

                        no_guide_found = false;
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
    last_var_guide_pos_ = -1;
    t_last_check_ = ros::Time::now().toSec();

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
                    
                    ROS_INFO("Guide has variance %f, last variance is %f", cov(0,0), last_var_guide_pos_);


                    //! Check if guide position is updated (initially negative)
                    if (cov(0,0) < last_var_guide_pos_ || last_var_guide_pos_ < 0) {
                        last_var_guide_pos_ = cov(0,0);
                        t_no_meas_ = 0;
                        t_last_check_ = ros::Time::now().toSec();
                    } else {

                        //! Uncertainty increased: guide out of side
                        last_var_guide_pos_ = cov(0,0);
                        t_no_meas_ += (ros::Time::now().toSec() - t_last_check_);
                        ROS_INFO("%f [s] without position update guide: ", t_no_meas_);

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
 * @brief speechCallback
 * @param res
 */
void speechCallback(std_msgs::String res) {

    //amigoSpeak(res.data);
    //if (res.data == "pleaseenterthelevator") {
    ROS_WARN("Received command: %s", res.data.c_str());
    //} else {
    //    ROS_WARN("Received unknown command \'%s\'", res.data.c_str());
    //}
}


/**
 * @brief moveTowardsPosition Let AMIGO move from its current position towards the given position
 * @param pos target position
 * @param offset, in case the position represents the guide position AMIGO must keep distance
 */
void moveTowardsPosition(pbl::PDF& pos, double offset, tf::TransformListener& tf_listener) {

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

    planner_->MoveToGoal(end_goal);

    ROS_INFO("Executive: Move base goal: (x,y,theta) = (%f,%f,%f)", end_goal.pose.position.x, end_goal.pose.position.y, theta);

}



int main(int argc, char **argv) {
    ros::init(argc, argv, "restaurant_simple");
    ros::NodeHandle nh;
    
     ROS_INFO("Started Restaurant");
    
    /// Head ref
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
    goal.head_tilt = 0.0;
    head_ref_pub.publish(goal);
    

    //! Planner
    planner_ = new CarrotPlanner("restaurant_carrot_planner");
    ROS_INFO("Carrot planner instantiated");

    //! Query WIRE for objects
    wire::Client client;
    ROS_INFO("Wire client instantiated");

    //! Client that allows reseting WIRE
    reset_wire_client_ = nh.serviceClient<std_srvs::Empty>("/wire/reset");

    //! Subscribe to the speech recognition topic
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/speech_recognition_restaurant/output", 10, speechCallback);

    //! Topic that makes AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/amigo_speak_up", 10);

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
    pbl::PDF guide_pos;

	findGuide(client, false);

    ROS_INFO("Found guide with position %s in frame \'%s\'", guide_pos.toString().c_str(), NAVIGATION_FRAME.c_str());
    amigoSpeak("Please show me the locations");

    //! Follow guide
    ros::Rate follow_rate(FOLLOW_RATE);
    while(ros::ok()) {
        ros::spinOnce();

        //! Get objects from the world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);


        //! Check if the robot arrived at itp two
        if (true) {

            //! Robot is asked to enter elevator and must leave when the guide leaves

            //! Find elevator
            //findAndEnterElevator(client, tf_listener);

        } else if (true) {

            //! Guide will pass through a small crowd of people (4-5) and calls the robot from behind the group

            //! Association will fail, hence this can be skipped. Check for the (updated) guide position
            if (getPositionGuide(objects, guide_pos)) {

                //! Move towards guide
                moveTowardsPosition(guide_pos, DISTANCE_GUIDE, tf_listener);


            } else {

                //! If the guide is lost, move around a crowd croud (wild guess)
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

                // Wild guess for guide
                findGuide(client, false);
            }



        } else {

            // Not at itp2/itp3, just follow guide

            //! Check for the (updated) guide position
            if (getPositionGuide(objects, guide_pos)) {

                //! Move towards guide
                moveTowardsPosition(guide_pos, DISTANCE_GUIDE, tf_listener);


            } else {

                //! Lost guide
                findGuide(client);
            }
        }

        follow_rate.sleep();
    }

    return 0;
}
