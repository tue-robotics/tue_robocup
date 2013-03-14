#include "wire_interface/Client.h"

#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me_simple");
    ros::NodeHandle nh;

    wire::Client client;

    string person_id;
    pbl::PDF person_pos;

    // wait until person is found
    ROS_INFO("Waiting until person is found ...");
    ros::Rate r(1);
    while(ros::ok() && person_id == "") {
        vector<wire::PropertySet> object_states = client.queryMAPObjects();
        for(vector<wire::PropertySet>::iterator it_obj = object_states.begin(); it_obj != object_states.end() && person_id == ""; ++it_obj) {
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
        r.sleep();
    }

    ROS_INFO("Found person with ID %s and position %s", person_id.c_str(), person_pos.toString().c_str());

    return 0;
}
