#include "wire_interface/Client.h"

#include <ros/ros.h>
#include <tue_move_base_msgs/MoveBaseAction.h>

// Action client
#include <actionlib/client/simple_action_client.h>

using namespace std;

//! World model frame
const string TRACKING_FRAME = "/base_link";

//! Rate at which the move base goal is updated
const double FOLLOW_RATE = 1;

//! Rate check for operator at start of the challenge
double FIND_RATE = 1;

//! Move base action client
actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>* move_base_;


void findOperator() {

}

void amigoSpeak(string sentence) {
    ROS_INFO("AMIGO: \'%s\'", sentence.c_str());
}


//! Get position and ID of the operator from the estimated world state
bool getPositionOperator(vector<wire::PropertySet>& objects, string& ID, pbl::PDF& pos) {

    for(vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end() && ID == ""; ++it_obj) {
        wire::PropertySet& obj = *it_obj;
        const wire::Property& prop_name = obj.getProperty("name");
        if (prop_name.isValid()) {
            if (prop_name.getValue().getExpectedValue().toString() == "operator") {
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid()) {
                    ID = obj.getID();
                    pos = prop_pos.getValue();
                    return true;
                } else {
                    ROS_WARN("Found a operator witour position");
                }
            }
        }
    }

    return false;
}


void memorizeOperator() {

    // TODO: no need to move arms? maybe instruction suffices

}


void moveTowardsPosition(pbl::PDF& pos) {

    pbl::Vector pos_exp = pos.getExpectedValue().getVector();

    tue_move_base_msgs::MoveBaseGoal move_base_goal;

    geometry_msgs::PoseStamped start;
    start.header.frame_id = TRACKING_FRAME;
    start.pose.position.x = 0;
    start.pose.position.y = 0;
    start.pose.position.z = 0;

    // set orientation
    // TODO: set correctly
    start.pose.orientation.x = 0;
    start.pose.orientation.y = 0;
    start.pose.orientation.z = 0;
    start.pose.orientation.w = 1;

    // First turn towards operator, then drive?
    move_base_goal.path.push_back(start);

    geometry_msgs::PoseStamped end_goal;
    end_goal.header.frame_id = TRACKING_FRAME;
    end_goal.pose.position.x = 0.75 * pos_exp(0);
    end_goal.pose.position.y = 0.75 * pos_exp(1);
    end_goal.pose.position.z = 0.75 * pos_exp(2);

    // set orientation
    // TODO: set correctly
    end_goal.pose.orientation.x = 0;
    end_goal.pose.orientation.y = 0;
    end_goal.pose.orientation.z = 0;
    end_goal.pose.orientation.w = 1;

    move_base_goal.path.push_back(end_goal);

    //! Send goal to move base client
    move_base_->sendGoal(move_base_goal);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me_simple");
    ros::NodeHandle nh;

    ROS_INFO("Started \'Follow me\'");

    //! Query WIRE for objects
    wire::Client client;

    //! Action client for sending move base goals
    move_base_ = new actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>("move_base",true);
    move_base_->waitForServer();

    ROS_INFO("Move base client connected to the move base server");

    //! Create a model for the operator
    memorizeOperator();

    //! Administration
    string person_id;
    pbl::PDF person_pos;

    //! Operator must be in the world model, remember position and ID
    ros::Rate find_rate(FIND_RATE);
    while(ros::ok() && person_id == "") {

        //! Get objects in estimated world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(TRACKING_FRAME);

        //! Start challenge once the operator is found
        if (getPositionOperator(objects, person_id, person_pos)) {
            break;
        }

        find_rate.sleep();
    }

    ROS_INFO("Found operator with ID %s and position %s (frame: %s)", person_id.c_str(), person_pos.toString().c_str(), TRACKING_FRAME.c_str());
    amigoSpeak("I am ready to start following you");

    //! Follow operator
    ros::Rate follow_rate(FOLLOW_RATE);
    while(ros::ok()) {

        //! Get objects from the world state
        vector<wire::PropertySet> objects = client.queryMAPObjects(TRACKING_FRAME);

        // TODO: check for ITP 2 (= elevator): sound input

        //! Check for the (updated) operator position
        if (getPositionOperator(objects, person_id, person_pos)) {

            //! Move towards operator
            moveTowardsPosition(person_pos);


        } else {

            //! Lost operator
            findOperator();
        }

        follow_rate.sleep();
    }

    return 0;
}
