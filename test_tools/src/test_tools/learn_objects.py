#! /usr/bin/env python

import sys

import rospy
import PyKDL as kdl


from robot_smach_states.util.designators import EntityByIdDesignator
from robot_skills.util.kdl_conversions import VectorStamped
from robot_skills.util.robot_constructor import robot_constructor


def look_at_entity(robot, location_des):
    # look on top of the surface
    entity = location_des.resolve()
    area = "on_top_of"
    search_volume = entity.volumes[area]
    x_obj = 0.5 * (search_volume.min_corner.x() + search_volume.max_corner.x())
    y_obj = 0.5 * (search_volume.min_corner.y() + search_volume.max_corner.y())
    z_obj = search_volume.min_corner.z()
    lookat_pos_map = entity.pose.frame * kdl.Vector(x_obj, y_obj, z_obj)
    x = lookat_pos_map.x()
    y = lookat_pos_map.y()
    z = lookat_pos_map.z()

    robot.head.look_at_point(VectorStamped(x, y, z, "/map"), timeout=0)
    rospy.loginfo("Looking at position ({}, {}, {})".format(x, y, z))
    robot.head.wait_for_motion_done()


def take_picture(robot, entity):
    res = robot.ed.update_kinect("{} {}".format("on_top_of", entity.id))
    segmented = res.new_ids + res.updated_ids
    robot.ed.classify(ids=segmented)


def learn_objects(robot, location):
    arm = robot.leftArm()
    if arm.has_joint_goal('arm_out_of_way'):
        rospy.loginfo('Getting arm out of the way')
        arm.send_joint_goal('arm_out_of_way')
        arm.wait_for_motion_done()

    look_at_entity(robot, location)

    entity = location.resolve()
    while not rospy.is_shutdown():
        count = 0
        while not rospy.is_shutdown():
            robot.speech.speak("Cheese")
            take_picture(robot, entity)
            rospy.sleep(1)

            count += 1
            if count > 10:
                break

            robot.speech.speak("move")  # instruct the user to move the objects
            rospy.sleep(1)

        robot.speech.speak("Switch")
        rospy.sleep(5)
        robot.speech.speak("clear")  #warn the user that a picture will be taken soon
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('learn_objects')

    if len(sys.argv) < 2:
        print("Please specify a robot name")
        sys.exit()

    robot_name = sys.argv[1]
    if robot_name != "hero":
        rospy.logwarn("Learn_objects is designed to be used with hero. It is unknown how it works for other robots")

    robot = robot_constructor(robot_name)

    location = "hero_case"
    location_des = EntityByIdDesignator(robot, location)

    learn_objects(robot, location_des)
