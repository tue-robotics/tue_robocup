#!/usr/bin/python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import sys

import robot_skills
from robot_smach_states import navigation

import robot_smach_states.util.designators as ds
from robot_smach_states.navigation import NavigateToPose, NavigateToObserve, NavigateToSymbolic, NavigateToWaypoint, \
    NavigateToRoom

if __name__ == "__main__":
    rospy.init_node('simple_navigate')

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        sys.exit()

    robot = Robot()

    if len(sys.argv) > 2:
        room = sys.argv[2]
    else:
        room = "kitchen"

    rospy.sleep(2) # wait for tf cache to be filled

    #nav_state = NavigateToPose(robot, 1, 0, 0)
    #nav_state.execute()

    #nav_state = NavigateToObserve(robot, ds.EntityByIdDesignator(robot=robot, id="dinnertable"))
    #nav_state.execute()

    # nav_state = NavigateToSymbolic(robot, {
    #                                         ds.EntityByIdDesignator(robot, id=room):"in"},
    #                                         ds.EntityByIdDesignator(robot, id=room))
    # nav_state.execute()

    #nav_state = NavigateToSymbolic(robot, {
    #                                        ds.EntityByIdDesignator(robot, id="couch"):"in_front_of"},
    #                                        ds.EntityByIdDesignator(robot, id="couch"))
    #nav_state.execute()

    # nav_state = NavigateToWaypoint(robot=robot, waypoint_designator=ds.EntityByIdDesignator(robot=robot,
    #                                                                                      id="hsr_demo_waypoint"),
    #                                radius=0.025)
    # nav_state.execute()

    nav_state = NavigateToRoom(robot=robot, entity_designator_room=ds.EntityByIdDesignator(robot=robot,
                                                                                           id="livingroom"))
    nav_state.execute()
