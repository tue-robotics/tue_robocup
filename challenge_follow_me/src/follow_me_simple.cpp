#include "wire_interface/Client.h"

#include <ros/ros.h>
#include <tue_move_base_msgs/MoveBaseAction.h>

// Action client
#include <actionlib/client/simple_action_client.h>

#include <pein_msgs/LearnAction.h>
#include <std_msgs/String.h>

#include <tf/transform_datatypes.h>
#include "problib/conversions.h"

using namespace std;


//! Settings
const string NAVIGATION_FRAME = "/base_link";   // Frame in which navigation goals are given IF NOT BASE LINK, UPDATE PATH IN moveTowardsPosition()
const double FOLLOW_RATE = 1;                   // Rate at which the move base goal is updated
double FIND_RATE = 1;                           // Rate check for operator at start of the challenge
const int N_MODELS = 40;                        // Rate check for operator at start of the challenge
const double TIME_OUT_LEARN_FACE = 90;          // Rate check for operator at start of the challenge
const double DISTANCE_OPERATOR = 1.0;           // Distance AMIGO keeps towards operator
const double THRESHOLD_POS_COV_OPERATOR = 2.4;  // If the position covariance of the operator is above the threshold: operator lost


//! Globals
bool itp2 = false;                                                                // Bookkeeping: at elevator yes or no
bool itp3 = false;                                                                // Bookkeeping: passed elevator yes or no
actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* move_base_ac_; // Move base action client
actionlib::SimpleActionClient<pein_msgs::LearnAction>* learn_face_ac_;            // Learn face action client
ros::Publisher pub_speech_;                                                       // Publisher that makes AMIGO speak


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
 * @brief findOperator, moves towards person closest to most recent operator position and turn on recognition
 * @param objects current list of world model objects
 * @param pos position the operator was seen last
 */
void findOperator(vector<wire::PropertySet>& objects, pbl::PDF& pos) {

    //! It is allowed to call the operator once per section (points for the section will be lost)
    amigoSpeak("I have lost my operator, can you please stand in front of me");

    //! Give the operator some time to move to the robot
    ros::Duration dt(7.5);
    dt.sleep();

    // TODO implement some search strategy (?)
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

                    //! Check if operator is lost based on uncertainty on its position
                    if (cov(0,0) > THRESHOLD_POS_COV_OPERATOR) {
                        ROS_INFO("Found operator but uncertainty too large -> lost operator");
                        return false;
                    }

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
    amigoSpeak("Please look at me while moving your head from the left to the right");

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


pbl::PDF findElevator() {

    pbl::PDF el_pos;
    return el_pos;

}


/**
 * @brief moveTowardsPosition Let AMIGO move from its current position towards the given position
 * @param pos target position
 * @param offset, in case the position represents the operator position AMIGO must keep distance
 */
void moveTowardsPosition(pbl::PDF& pos, double offset) {

    pbl::Vector pos_exp = pos.getExpectedValue().getVector();

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

    //! End point of the path is the given position
    geometry_msgs::PoseStamped end_goal;
    end_goal.header.frame_id = NAVIGATION_FRAME;
    q.setRPY(pos_exp(0), pos_exp(1), 0);

    //! Set orientation
    end_goal.pose.orientation.x = q.getX();
    end_goal.pose.orientation.y = q.getY();
    end_goal.pose.orientation.z = q.getZ();
    end_goal.pose.orientation.w = q.getW();

    //! Incorporate offset in target position
    double full_distance = sqrt(pos_exp(0)*pos_exp(0)+pos_exp(1)*pos_exp(1));
    double reduced_distance = full_distance - offset;
    end_goal.pose.position.x = pos_exp(0) * reduced_distance / full_distance;
    end_goal.pose.position.y = pos_exp(1) * reduced_distance / full_distance;
    end_goal.pose.position.z = 0;
    move_base_goal.path.push_back(end_goal);

    //! Send goal to move base client
    move_base_ac_->sendGoal(move_base_goal);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me_simple");
    ros::NodeHandle nh;

    ROS_INFO("Started Follow me");

    //! Query WIRE for objects
    wire::Client client;

    //! Subscribe to the speech recognition topic
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/speech_recognition_follow_me/output", 10, speechCallback);
    bool itp2 = false;

    //! Topic that makes AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/amigo_speak_up", 10);

    //! Action client for sending move base goals
    move_base_ac_ = new actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>("move_base",true);
    move_base_ac_->waitForServer();
    ROS_INFO("Move base client connected to the move base server");

    //! Action client for learning faces
    learn_face_ac_ = new actionlib::SimpleActionClient<pein_msgs::LearnAction>("/face_learning/action_server",true);
    learn_face_ac_->waitForServer();
    ROS_INFO("Learn face client connected to the learn face server");

    //! Create a model for identifying the operator later
    if (!memorizeOperator()) {
        ROS_ERROR("Learning operator failed: AMIGO does not know who to follow");
        return -1;
    }

    //! Administration
    pbl::PDF operator_pos;

    //! Operator must be in the world model, remember position and ID
    ros::Rate find_rate(FIND_RATE);
    while(ros::ok()) {

        //! Get objects in estimated world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);

        //! Start challenge once the operator is found
        if (getPositionOperator(objects, operator_pos)) {
            break;
        }

        ROS_INFO("No operator found, waiting for operator...");
        find_rate.sleep();
    }

    ROS_INFO("Found operator with position %s in frame \'%s\'", operator_pos.toString().c_str(), NAVIGATION_FRAME.c_str());
    amigoSpeak("I will now start following you");

    //! Follow operator
    ros::Rate follow_rate(FOLLOW_RATE);
    while(ros::ok()) {

        //! Get objects from the world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(NAVIGATION_FRAME);

        //! Check if the robot arrived at itp two
        if (itp2) {

            //! Robot is asked to enter elevator and must leave when the operator leaves

            // TODO: find elevator
            pbl::PDF elevator_pos = findElevator();

            //! Enter elevator
            moveTowardsPosition(elevator_pos, 0);

            // TODO: find operator and wait for operator to leave

            itp2 = false;
            itp3 = true;

        } else if (itp3) {

            //! Operator will pass through a small crowd of people (4-5) and calls the robot from behind the group

            //! Check for the (updated) operator position
            if (getPositionOperator(objects, operator_pos)) {

                //! Move towards operator
                moveTowardsPosition(operator_pos, DISTANCE_OPERATOR);


            } else {

                //! If the operator is lost, move around the croud

                // TODO: drive around croud
                // TODO: somehow the operator must be recognized again

                itp3 = false;
            }



        } else {

            // Not at itp2/itp3, just follow operator

            //! Check for the (updated) operator position
            if (getPositionOperator(objects, operator_pos)) {

                //! Move towards operator
                moveTowardsPosition(operator_pos, DISTANCE_OPERATOR);


            } else {

                //! Lost operator
                findOperator(objects, operator_pos);
            }
        }

        follow_rate.sleep();
    }

    return 0;
}
