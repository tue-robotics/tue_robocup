/*
 * test_follower.cpp
 *
 *  Created on: Feb 17, 2014
 *      Author: Jos Elfring
 */

#include <ros/ros.h>
#include "challenge_follow_me/Follower.h"


int main(int argc, char **argv) {

	ros::init (argc, argv, "test_follower");
	ros::NodeHandle nh;

	ROS_INFO("Started test node for the Follower class");

    // true: gmapping, false: carrot planner
    bool use_map = false;
    Follower fol(nh, "/amigo/base_link", use_map);
    if (!fol.start())
    {
        return -1;
    }

    ros::Rate r(20);
    while (nh.ok())
    {
        // Is the spin once needed?
        ros::spinOnce();

        // Update
        bool ok = fol.update();
        if (!ok) ROS_WARN("Could not update Follower");

        // Sleep
        r.sleep();
    }

	return 0;

}
