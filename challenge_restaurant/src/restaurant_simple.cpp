#include "wire_interface/Client.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <tue_move_base_msgs/MoveBaseAction.h>

// Action client
#include <actionlib/client/simple_action_client.h>

#include <pein_msgs/LearnAction.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include "perception_srvs/StartPerception.h"

#include "problib/conversions.h"

#include "tue_carrot_planner/carrot_planner.h"
#include <amigo_msgs/head_ref.h>

#include <text_to_speech_philips/Speak.h>

// Speech recognition
#include "tue_pocketsphinx/Switch.h"

using namespace std;

// Possible speech commands
// amigostop
// thislocationisnamed


//! Settings
const int TIME_OUT_GUIDE_LOST = 8;              // Time interval without updates after which operator is considered to be lost
const double DISTANCE_GUIDE = 0.8;              // Distance AMIGO keeps towards guide
const double WAIT_TIME_GUIDE_MAX = 15.0;        // Maximum waiting time for guide to return
const string NAVIGATION_FRAME = "/base_link";   // Frame in which navigation goals are given IF NOT BASE LINK, UPDATE PATH IN moveTowardsPosition()
const double FOLLOW_RATE = 20;                  // Rate at which the move base goal is updated
double FIND_RATE = 5;                           // Rate check for guide at start of the challenge

// NOTE: At this stage recognition is never performed, hence number of models can be small


//! Globals
CarrotPlanner* planner_;
double t_no_meas_ = 0;                                                            // Bookkeeping: determine how long guide is not observed
double t_last_check_ = 0;                                                         // Bookkeeping: last time guide position was checked
double last_var_guide_pos_ = -1;                                                  // Bookkeeping: last variance in x-position guide
actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* move_base_ac_; // Communication: Move base action client
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak
ros::ServiceClient srv_speech_;                                                   // Communication: Service that makes AMIGO speak
ros::ServiceClient reset_wire_client_;                                            // Communication: Client that enables reseting WIRE
bool freeze_amigo_ = false;                                                       // Bookkeeping: true if stop command is confirmed
bool candidate_freeze_amigo_ = false;                                             // Bookkeeping: true if robot is asked to stop
ros::Publisher head_ref_pub_;                                                     // Communication: Look to intended driving direction
unsigned int n_locations_learned_ = 0;                                            // Bookkeeping: number of locations learned
ros::ServiceClient speech_recognition_client_;                                    // Communication: Client for starting / stopping speech recognition
double t_freeze_ = 0;


/**
 * @brief amigoSpeak let AMIGO say a sentence
 * @param sentence
 */
void amigoSpeak(string sentence) {
	
	ROS_INFO("AMIGO: \'%s\'", sentence.c_str());

    
    //! Call speech service
	text_to_speech_philips::Speak speak;
	speak.request.sentence = sentence;
    speak.request.language = "us";
    speak.request.character = "kyle";
    speak.request.voice = "default";
    speak.request.emotion = "normal";
    speak.request.blocking_call = true;

    if (!srv_speech_.call(speak)) {
		std_msgs::String sentence_msgs;
		sentence_msgs.data = sentence;
		pub_speech_.publish(sentence_msgs);
	}

}



/**
 * @brief findGuide, detects person in front of robot, empties WIRE and adds person as guide
 * @param client used to query from/assert to WIRE
 */
bool findGuide(wire::Client& client, bool lost = true) {

    //! It is allowed to call the guide once per section (points for the section will be lost)
    if (lost) {
        amigoSpeak("I have lost my guide, can you please stand in front of me");
    }

    //! Give the guide some time to move to the robot
    ros::Duration wait_for_guide(6.0);
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

            //! Find person that has smallest Eucledian distance to robot
            if (vector_possible_guides.size() > 1) {

                double dist_min = 0.0;
                int i_best = 0;

                for (unsigned int i = 0; i < vector_possible_guides.size(); ++i)
                {
                    double dx = vector_possible_guides[i].getMean()(1);
                    double dy = vector_possible_guides[i].getMean()(0);
                    double dist = sqrt(dx*dx+dy*dy);
                    if (i == 0 || dist_min > dist)
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

            //! Assert evidence to WIRE
            client.assertEvidence(ev);

            no_guide_found = false;

            amigoSpeak("I found my guide. Hi guide. I will follow you");

            ros::Duration safety_delta(1.0);
            safety_delta.sleep();
            return true;

        }

        dt.sleep();
    }
    
    amigoSpeak("I did not find my guide yet");
    return false;

}

bool startSpeechRecognition() {
    std::string knowledge_path = ros::package::getPath("tue_knowledge");
    if (knowledge_path == "") {
        return false;
    }

    std::string follow_me_speech_path = knowledge_path + "/speech_recognition/restaurant/";

    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::START;
    req.hidden_markov_model = "/usr/share/pocketsphinx/model/hmm/wsj1";
    req.dictionary = follow_me_speech_path + "restaurant.dic";
    req.language_model = follow_me_speech_path + "restaurant.lm";

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
                        ROS_INFO("Time since last update was %f", ros::Time::now().toSec()-t_last_check_);
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

    ROS_INFO("Received command: %s", res.data.c_str());

    if (res.data == "amigostop") {
        candidate_freeze_amigo_ = true;
        amigoSpeak("Do you want me to stop?");
        t_freeze_ = ros::Time::now().toSec();
    } else if (candidate_freeze_amigo_ &&  res.data == "yes") {
		freeze_amigo_ = true;
		candidate_freeze_amigo_ = false;
		amigoSpeak("I will remember this location. How is it called?");
    } else if (candidate_freeze_amigo_ && res.data != "yes" && res.data != "") {
		candidate_freeze_amigo_ = false;
		amigoSpeak("Sorry. I misunderstood. I will follow you");	
	//} else if (freeze_amigo_ && (res.data == "thislocationisnamed" || res.data == "thislocationiscalled")) {
    } else if (res.data.find("thislocationisnamed") != std::string::npos || res.data.find("thislocationiscalled") != std::string::npos) {
        freeze_amigo_ = false;
        ros::Duration delta(2.0);
        delta.sleep();
        amigoSpeak("Thank you. I will now continue to follow you");
        ++n_locations_learned_;
    }

    // always immediately start listening again
    startSpeechRecognition();
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

    amigo_msgs::head_ref goal;
    goal.head_pan = theta;
    goal.head_tilt = 0.0;
    head_ref_pub_.publish(goal);

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



int main(int argc, char **argv) {
    ros::init(argc, argv, "restaurant_simple");
    ros::NodeHandle nh;
    
     ROS_INFO("Started Restaurant");
    
    /// Head ref
    head_ref_pub_ = nh.advertise<amigo_msgs::head_ref>("/head_controller/set_Head", 1);
    
    /// set the head to look down in front of AMIGO
    ros::Rate poll_rate(100);
    while (head_ref_pub_.getNumSubscribers() == 0) {
        ROS_INFO_THROTTLE(1, "Waiting to connect to head ref topic...");
        poll_rate.sleep();
    }
    ROS_INFO("Sending head ref goal");
    amigo_msgs::head_ref goal;
    goal.head_pan = 0.0;
    goal.head_tilt = 0.0;
    head_ref_pub_.publish(goal);
    
    //! Planner
    double max_vel_lin = 0.4;   // Default: 0.50
    double max_vel_ang = 0.3;   // Default: 0.40
    double dist_to_wall = 0.45; // Default: 0.65
    planner_ = new CarrotPlanner("restaurant_carrot_planner", max_vel_lin, max_vel_ang, dist_to_wall);
    ROS_INFO("Carrot planner instantiated");

    //! Switch on perception
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

    //! Query WIRE for objects
    wire::Client client;
    ROS_INFO("Wire client instantiated");

    //! Client that allows reseting WIRE
    reset_wire_client_ = nh.serviceClient<std_srvs::Empty>("/wire/reset");
    ROS_INFO("Service /wire/reset");

    // Start speech recognition
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 10, speechCallback);
    startSpeechRecognition();


    //! Topic/srv that make AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/text_to_speech/input", 10);
    srv_speech_ =  nh.serviceClient<text_to_speech_philips::Speak>("/text_to_speech/speak");
    ROS_INFO("Publisher/service client for text to speech started");
    
    //! Always clear the world model
    std_srvs::Empty srv;
    if (reset_wire_client_.call(srv)) {
        ROS_INFO("Cleared world model");
    } else {
        ROS_ERROR("Failed to clear world model");
    }

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
    while(ros::ok()) {
        ros::spinOnce();

        //! Get objects from the world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);

        if (n_locations_learned_ == 5) {
            ROS_WARN("I think I know all locations");
        }


        //! Check if the robot has to move
        if (freeze_amigo_) {
			
			
            // TODO: Check if this function is needed
			bool no_use = getPositionGuide(objects, guide_pos);

            //! Robot is waiting for the name and will then continue following
            pbl::Matrix cov(3,3);
            cov.zeros();
            pbl::PDF pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
            moveTowardsPosition(pos, 0);
            

        } else {

            // Just follow
            if (candidate_freeze_amigo_) {
	            ROS_INFO("Time out is %f", ros::Time::now().toSec() - t_freeze_);
	            if (ros::Time::now().toSec() - t_freeze_ > 4) {
					ROS_WARN("Time out");
					candidate_freeze_amigo_ = false;
				}
			}

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

    //! When node is shut down, cancel goal by sending zero
    pbl::Matrix cov(3,3);
    cov.zeros();
    pbl::PDF pos = pbl::Gaussian(pbl::Vector3(0, 0, 0), cov);
    moveTowardsPosition(pos, 0);

    return 0;
}
