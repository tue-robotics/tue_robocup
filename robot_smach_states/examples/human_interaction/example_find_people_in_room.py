#! /usr/bin/env python

import sys
import rospy
import robot_smach_states.util.designators as ds
from robot_skills.util.entity import Entity
from robot_smach_states.human_interaction import FindPeopleInRoom

if __name__ == "__main__":
    from robot_skills import get_robot

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
        _area = sys.argv[2]

        rospy.init_node('example_find_people_in_room')
        _robot = get_robot(robot_name)

        people = ds.VariableDesignator(resolve_type=[Entity])
        sm = FindPeopleInRoom(_robot, _area, people.writeable)
        sm.execute()

        rospy.loginfo("Found {}".format(people.resolve()))
    else:
        print("Please provide robot name as argument.")
        exit(1)
