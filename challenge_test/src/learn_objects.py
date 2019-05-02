#! /usr/bin/env python

#TODO this file does not belong in challenge_test move it somewhere that makes sense

import rospy
import PyKDL as kdl


from robot_smach_states.util.designators import EntityByIdDesignator
from robot_skills.util.kdl_conversions import VectorStamped


def look_at_entity(robot, location_des):
    #look on top of the surface
    entity = location_des.resolve()
    area  = "on_top_of"  #TODO make the area variable and/or a designator
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
    rospy.loginfo("done looking a the {}".format(entity.id))


if __name__ == '__main__':
    rospy.init_node('learn_objects_exec')

    #TODO make this so that it also works for AMIGO and SERGIO
    from robot_skills import Hero as Robot
    robot = Robot()

    #TODO make table entity variable
    location = "hero_case"
    location_des = EntityByIdDesignator(robot, location)

    arm = robot.leftArm()
    arm.send_joint_goal('arm_out_of_way')
    arm.wait_for_motion_done()
    #robot.leftArm().send_joint_goal("arm_out_of_way")
    #robot.leftArm().wait_for_motion_done()

    robot.torso.high()
    robot.torso.wait_for_motion_done()

    look_at_entity(robot, location_des)

    entity = location_des.resolve()
    while not rospy.is_shutdown():
        count = 0
        while not rospy.is_shutdown() and count < 10:
            robot.ed.update_kinect("{} {}".format("on_top_of", entity.id))
            rospy.sleep(1)
            count += 1
        robot.speech.speak("Pause")
        rospy.sleep(3)


