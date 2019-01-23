#! /usr/bin/env python

from robot_skills.amigo import Amigo
import PyKDL as kdl
import rospy

if __name__ == '__main__':
    rospy.init_node("table_updater", anonymous=True)

    amigo = Amigo()

    table = amigo.ed.get_entity(id="dinner_table")
    pp = table.pose

    r = rospy.Rate(0.1)

    i=0
    while not rospy.is_shutdown():
        pp.frame.M = kdl.Rotation.EulerZYX(i * 0.1, 0, 0)
        amigo.ed.update_entity(id="dinner_table", frame_stamped=pp)
        i += 1
        r.sleep()

