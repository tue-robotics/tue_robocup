#include "wire_interface/Client.h"

#include <ros/ros.h>
#include <tue_move_base_msgs/MoveBaseAction.h>

// Action client
#include <actionlib/client/simple_action_client.h>

using namespace std;

string TRACKING_FRAME = "/base_link";

double FIND_RATE = 1;
double FOLLOW_RATE = 1;

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me_simple");
    ros::NodeHandle nh;

    wire::Client client;

    actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction> move_base("move_base", true);
    move_base.waitForServer();

    string person_id;
    pbl::PDF person_pos;

    // wait until person is found
    ROS_INFO("Waiting until person is found ...");

    ros::Rate find_rate(FIND_RATE);
    while(ros::ok() && person_id == "") {
        vector<wire::PropertySet> objects = client.queryMAPObjects(TRACKING_FRAME);
        for(vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end() && person_id == ""; ++it_obj) {
            wire::PropertySet& obj = *it_obj;
            const wire::Property& prop_class = obj.getProperty("class_label");
            if (prop_class.isValid()) {
                if (prop_class.getValue().getExpectedValue().toString() == "person") {
                    const wire::Property& prop_pos = obj.getProperty("position");
                    if (prop_pos.isValid()) {
                        person_id = obj.getID();
                        person_pos = prop_pos.getValue();
                    } else {
                        ROS_WARN("Found a person WITHOUT position (ID: %s)", obj.getID().c_str());
                    }
                }
            }
        }
        find_rate.sleep();
    }

    ROS_INFO("Found person with ID %s and position %s (frame: %s)", person_id.c_str(), person_pos.toString().c_str(), TRACKING_FRAME.c_str());

    // follow person
    ROS_INFO("Following person with ID %s", person_id.c_str());

    ros::Rate follow_rate(FOLLOW_RATE);
    while(ros::ok()) {
        vector<wire::PropertySet> objects = client.queryMAPObjects(TRACKING_FRAME);
        for(vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
            wire::PropertySet& obj = *it_obj;
            if (obj.getID() == person_id) {
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid()) {
                    person_pos = prop_pos.getValue();
                }
            }
        }

        // get expected position
        if (person_pos.getExpectedValue().isValid()) {
            pbl::Vector pos_exp = person_pos.getExpectedValue().getVector();

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

            // send to move_base
            move_base.sendGoal(move_base_goal);

        } else {
            ROS_WARN("Could not determine most likely position from PDF: %s", person_pos.toString().c_str());
        }

        follow_rate.sleep();
    }

    return 0;
}
